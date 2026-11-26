import math

import cv2
import numpy as np
import pytest

from roki_ng.detection.football import goal_parts
from roki_ng.detection.football import assemble_goals
from roki_ng.detection.football import render_scene
from roki_ng.parameters import Parameters


class Projector:
    def _pixel_ray(self, pixel, quaternion):
        u, v = pixel
        return np.array([1., (400-u)/700, (216.6667-v)/700])


def picture(tmp_path, left=True, right=True, bar=True, cropped=False):
    p = Parameters(tmp_path).values
    for i in (0, 1):
        p[f'field.goal.{i}']['width'] = .7
    colours = {'blue_posts': (255, 0, 0), 'green_field': (0, 150, 0),
               'white_marking': (255, 255, 255), 'white_posts': (255, 255, 255),
               'yellow_posts': (0, 255, 255)}
    for name, bgr in colours.items():
        l, a, b = map(float, cv2.cvtColor(np.uint8([[bgr]]), cv2.COLOR_BGR2LAB)[0, 0])
        for key, value in dict(l_min=max(0, l/2.55-2), l_max=min(100, l/2.55+2),
                               a_min=a-128-2, a_max=a-128+2, b_min=b-128-2, b_max=b-128+2).items():
            p[f'vision.{name}.{key}'] = value
    image = np.zeros((650, 800, 3), np.uint8)
    image[450:] = colours['green_field']
    if left:
        cv2.rectangle(image, (200, 100), (220, 449), colours['blue_posts'], -1)
    if right:
        cv2.rectangle(image, (600, 100), (620, 449), colours['blue_posts'], -1)
    if bar:
        cv2.rectangle(image, (0 if cropped else 200, 100), (620, 112), colours['blue_posts'], -1)
    return image, p


@pytest.mark.parametrize('left,right,bar', [(True, True, False), (True, False, True), (False, True, True)])
def test_goal_detector_accepts_actual_two_element_images(tmp_path, left, right, bar):
    image, p = picture(tmp_path, left, right, bar)
    parts = goal_parts(image, p, Projector(), (), dict(height=.4, x=0., y=0., pan=0.))
    goals = assemble_goals(parts, [p['field.goal.1']])
    assert len(goals) == 1
    assert goals[0]['elements'] == ('opening' if left and right else 'colour only')
    assert goals[0]['shootable'] == (left and right)


def test_cropped_bar_cannot_replace_missing_post(tmp_path):
    image, p = picture(tmp_path, left=False, right=True, bar=True, cropped=True)
    parts = goal_parts(image, p, Projector(), (), dict(height=.4, x=0., y=0., pan=0.))
    assert not any(goal['shootable'] for goal in assemble_goals(parts, [p['field.goal.1']]))


def test_diagnostic_does_not_modify_raw_frame(tmp_path):
    image, _ = picture(tmp_path)
    before = image.copy()
    out = render_scene(image, None, 'kick forbidden: no confirmed goal')
    assert np.array_equal(before, image)
    assert out.shape == image.shape and not np.array_equal(out, image)


@pytest.mark.parametrize('bearing_only', [False, True])
def test_goal_behind_line_and_false_yellow_edges(tmp_path, bearing_only):
    image, p = picture(tmp_path)
    image[450:478] = (65, 65, 65)  # bare floor between uprights and pitch
    cv2.line(image, (0, 570), (160, 475), (0, 255, 255), 4)
    cv2.rectangle(image, (90, 110), (104, 210), (0, 255, 255), -1)
    geometry = dict(height=.4, x=0., y=0., pan=0.)
    def seen():
        return {(v['colour'], v['kind']) for v in
                goal_parts(image, p, Projector(), (), geometry, bearing_only=bearing_only)}
    assert seen() == ({('blue', 'colour')} if bearing_only else {('blue', 'colour'), ('blue', 'opening')})
    image[100:450, 200:621][np.all(image[100:450, 200:621] == (255,0,0), axis=2)] = (0,255,255)
    assert seen() == ({('yellow', 'colour')} if bearing_only else {('yellow', 'colour'), ('yellow', 'opening')})


def test_fragmented_goal_with_bases_at_different_heights(tmp_path):
    image, p = picture(tmp_path, left=False, right=False, bar=False)
    image[350:] = (0, 150, 0)
    cv2.line(image, (210, 160), (610, 160), (255, 0, 0), 12)
    cv2.line(image, (210, 160), (210, 430), (255, 0, 0), 12)
    cv2.line(image, (610, 160), (610, 350), (255, 0, 0), 12)
    # Shadow gaps interrupt the colour mask but not the physical tube.
    for y in range(180, 340, 12):
        image[y:y+3, 603:618] = 0
    parts = goal_parts(image, p, Projector(), (), dict(height=.4, x=0., y=0., pan=0.))
    openings = [part for part in parts if part['kind'] == 'opening']
    assert len(openings) == 1
    import msgpack
    msgpack.packb(parts)
    left, right = openings[0]['feet_px']
    assert abs(left[0]-210) < 12 and abs(right[0]-610) < 12
    assert left[1]-right[1] > 60


def test_floor_colour_stroke_is_not_an_upright(tmp_path):
    image, p = picture(tmp_path)
    # Yellow-green fringe of the white perimeter, surrounded by floor/turf.
    cv2.rectangle(image, (40, 470), (45, 590), (0, 255, 255), -1)
    cv2.rectangle(image, (29, 470), (39, 590), (255, 255, 255), -1)
    parts = goal_parts(image, p, Projector(), (), dict(height=.4, x=0., y=0., pan=0.))
    assert not any(part['colour'] == 'yellow' for part in parts)
    assert any(part['colour'] == 'blue' and part['kind'] == 'opening' for part in parts)


def test_occluded_goal_line_needs_paint_on_both_flanks():
    from roki_ng.detection.football import goal_line_gaps
    paint = np.zeros((650,800),np.uint8)
    cv2.line(paint,(120,412),(520,390),255,4)
    paint[:,270:340] = 0
    gaps = goal_line_gaps(paint,(120,410),(520,390))
    assert len(gaps)==1
    assert gaps[0] == pytest.approx([.375,.5375],abs=.025)
    paint[:,:340] = 0
    assert goal_line_gaps(paint,(120,410),(520,390)) == []


def test_penalty_rectangle_bridges_blocked_front_when_a_return_is_visible():
    from roki_ng.detection.football import goal_box_support
    class Camera:
        height = .4
        def image_points(self, xy, q):
            uv = np.column_stack((400-230*xy[:,1],600-230*xy[:,0]))
            return uv, np.ones(len(uv), bool)
    profile = {'field.goal.0': dict(x=1.7,y=0.,colour='blue',width=1.),
               'field.mark.02': dict(enabled=True,kind='line',x=1.4,y=0.,angle=math.pi/2,size=1.2),
               'field.mark.03': dict(enabled=True,kind='line',x=1.55,y=.6,angle=0.,size=.3)}
    paint = np.zeros((650,800),bool)
    paint[275:281,262:539] = True
    paint[207:279,260:266] = True
    paint[:,357:444] = False
    args = ([[1.7,.5],[1.7,-.5]],profile,'blue',paint,Camera(),None,dict(x=0.,y=0.,pan=0.,height=.4))
    assert goal_box_support(*args)
    paint[:274] = False
    assert not goal_box_support(*args)
