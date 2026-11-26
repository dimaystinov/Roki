"""Robot-relative football observations; no map or accumulated robot pose."""
import math
import time
from itertools import combinations

from roki_ng.body.calibration import wrap

TICK = .00058909


def camera_geometry(p, pan, tilt, model):
    from roki_ng.vision.camera_geometry import height
    neck = model.neck_m
    ref = model.reference_tilt*TICK
    angle, pitch = model.pan_sign*pan*TICK, tilt*TICK
    radius = neck*math.sin(-pitch)
    optical_height = height(p, model)
    if p.get('camera.custom_height_m', 0.) == 0:
        optical_height += neck*(math.cos(pitch)-math.cos(ref))
    return dict(height=optical_height,
                x=model.forward_m-neck*math.sin(-ref)+radius*math.cos(angle),
                y=radius*math.sin(angle), pan=angle)


def body_point(point, geometry):
    x, y = point
    c, s = math.cos(geometry['pan']), math.sin(geometry['pan'])
    return [geometry['x']+c*x-s*y, geometry['y']+s*x+c*y]


def plane_point(projector, pixel, q, geometry, z=0.):
    ray = projector._pixel_ray(pixel, q)
    if abs(float(ray[2])) < .04:
        raise ValueError('Ray near horizon')
    scale = (z-geometry['height'])/float(ray[2])
    if scale <= 0:
        raise ValueError('Plane behind camera')
    point = body_point((float(ray[0])*scale, float(ray[1])*scale), geometry)
    if math.hypot(*point) > 6:
        raise ValueError('Object beyond usable range')
    return point


def ball_at_foot(image, rect):
    """Use the two dark toe tips in a centred downward view of the Roki2 body."""
    import cv2
    import numpy as np
    h, w = image.shape[:2]
    top = round(.53*h)
    crop = image[top:round(.76*h), round(.33*w):round(.65*w)]
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    dark = cv2.inRange(hsv, (0, 0, 0), (179, 115, 86))
    dark = cv2.morphologyEx(dark, cv2.MORPH_OPEN,
                            np.ones((max(2, h//100), max(3, w//80)), np.uint8))
    profile = np.where(dark.any(axis=0), dark.argmax(axis=0), len(dark))
    baseline = np.percentile(profile, 80)
    raised = (profile < baseline - h*.014).astype(np.int8)
    raised = cv2.morphologyEx(raised[None, :].astype(np.uint8), cv2.MORPH_CLOSE,
                              np.ones((1, max(3, w//80)), np.uint8))[0]
    edges = np.where(np.diff(np.pad(raised, (1, 1))))[0]
    tips = []
    for a, b in zip(edges[::2], edges[1::2]):
        if .045*w <= b-a <= .105*w:
            tips.append([round(.33*w)+int(a), top+int(np.median(profile[a:b])),
                         round(.33*w)+int(b)])
    if len(tips) != 2:
        return None
    left, right = tips
    if not (.08*w < right[0]-left[0] < .17*w and abs(left[1]-right[1]) < .035*h):
        return None
    x1, y1, x2, y2 = rect
    cx = (x1+x2)/2
    leg, tip = min(zip(('left','right'), tips), key=lambda item: abs(cx-(item[1][0]+item[1][2])/2))
    diameter = max(1, y2-y1)
    gap = (tip[1]-y2)/diameter
    return dict(leg=leg, aligned=tip[0] <= cx <= tip[2], gap_ratio=round(gap, 3),
                tips=tips, ball_rect=list(rect))


def _supported_uprights(raw, labels, turf, stats=None):
    """Find upright colour strokes at or just beyond the edge of the visible turf."""
    import cv2
    import numpy as np
    shafts = raw.copy()
    if stats is not None:
        for i, (x, y, w, h, _) in enumerate(stats[1:], 1):
            if w >= h and h >= 45:
                # The upper bar can be thicker than the morphology kernel close up.
                band = np.s_[y:y+h//4, x:x+w]
                shafts[band][labels[band] == i] = 0
    strokes = cv2.morphologyEx(shafts, cv2.MORPH_OPEN, np.ones((17, 3), np.uint8),
                               borderType=cv2.BORDER_CONSTANT, borderValue=0)
    _, ids, stats, _ = cv2.connectedComponentsWithStats(strokes)
    supported = {}
    for i, (x, y, w, h, _) in enumerate(stats[1:], 1):
        if h < 1.3*w or y+h >= raw.shape[0]:
            continue
        bottom = np.flatnonzero(ids[y+h-1, x:x+w] == i)
        u = x+int(bottom[len(bottom)//2])
        foot = y+h
        # A post can stand behind the goal line on bare floor. Search below
        # its foot, allowing a perspective-scaled gap without classifying it as turf.
        gap_limit = max(8, min(48, round(h*.35)))
        green = turf[foot:foot+gap_limit+8, max(0,u-5):u+6]
        for gap in range(0, gap_limit+1, 4):
            if len(green) >= gap+8 and green[gap:gap+8].mean() >= .5:
                supported.setdefault(int(labels[foot-1,u]), []).append((int(u),int(foot-1)))
                break
    return supported


def goal_line_gaps(paint, left, right):
    """Interpolate the ground line only between visible paint on both flanks."""
    import numpy as np
    t = np.linspace(0.,1.,81)
    x,y = np.rint(np.asarray(left)[:,None]+(np.asarray(right)-left)[:,None]*t).astype(int)
    # Fit the stroke between the two bases. A broad vertical union would
    # join paint to bright hardware and hide the interruption at a keeper.
    best_score = -1.
    support = np.zeros(len(t),bool)
    for dy_left in (-5,-2,1,4,7):
        for dy_right in (-5,-2,1,4,7):
            v = y + np.rint(dy_left*(1-t)+dy_right*t).astype(int)
            paint_here = paint[v.clip(0,paint.shape[0]-1),x.clip(0,paint.shape[1]-1)] != 0
            score = paint_here.sum() - .06*(abs(dy_left)+abs(dy_right))
            if score > best_score:
                support,best_score = paint_here,score
    # Small white reflections on a robot's shins can punctuate the gap.
    import cv2
    support = cv2.morphologyEx(support.astype('uint8').reshape(1,-1), cv2.MORPH_OPEN,
                               np.ones((1,3),np.uint8)).ravel()
    local = np.convolve(np.pad(support.astype(float),(2,2),mode='edge'), np.ones(5)/5,mode='valid')
    support = local >= .75
    changes = np.diff(np.r_[True,support,True].astype(int))
    gaps = []
    for lo,hi in zip(np.flatnonzero(changes==-1),np.flatnonzero(changes==1)):
        if (.05 <= (hi-lo)/80 <= .38 and lo>=5 and hi+5<=len(t)
                and support[lo-5:lo].all() and support[hi:hi+5].all()):
            gaps.append([float(t[lo]),float(t[hi-1])])
    return gaps


def keeper_span(image, left, right, rect):
    """Team-coloured chest above dark legs, wholly between supported goalposts."""
    import cv2
    import numpy as np
    x0, x1 = round(left[0]), round(right[0])
    bottom = round(max(left[1], right[1]))
    w, h = x1-x0, bottom-rect[1]
    if w < 40 or h < 32:
        return []
    crop = image[rect[1]:bottom, x0:x1]
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    hue, sat, val = cv2.split(hsv)
    bib = (((hue < 10) | (hue > 170) | ((hue >= 100) & (hue <= 135)))
           & (sat > 150) & (val > 65)).astype(np.uint8)
    _, _, stats, _ = cv2.connectedComponentsWithStats(bib)
    candidates = []
    for x, y, bw, bh, area in stats[1:]:
        if not (.05*w < bw < .25*w and .035*h < bh < .19*h and
                1.2 < bw/bh < 3.8 and .23*h < y < .73*h and area > .63*bw*bh):
            continue
        base = round(left[1] + (right[1]-left[1])*(x+bw/2)/w) - rect[1]
        # A person's orange shoes can overlap the mouth in projection. Require
        # neutral dark robot legs reaching the goal line below the bib.
        dark = (val < 105) & (sat < 95)
        feet = dark[max(y+bh, base-round(.12*h)):min(h,base), x:x+bw]
        legs = dark[y+bh:min(h,y+bh*4), x:x+bw]
        if feet.size and feet.mean() > .25 and legs.size and legs.mean() > .27:
            candidates.append([float(max(0.,(x-.48*bw)/w)), float(min(1.,(x+1.48*bw)/w))])
    return candidates

def occupied_goal_lanes(lab, left, right, rect):
    """Estimate foreground spans across the lower goal mouth, in image order."""
    import cv2
    import numpy as np
    width = right[0] - left[0]
    height = max(left[1], right[1]) - rect[1]
    if width < 24 or height < 24:
        return [[0., 1.]]
    # Sample at proportional heights above the sloping line between the feet.
    # The two strips just outside the posts supply a local background reference.
    t = np.linspace(-.30, 1.30, 161)
    xx = np.rint(left[0]+width*t).astype(int)
    yy = left[1]+(right[1]-left[1])*t
    rows = np.linspace(.12, .33, 18)
    y = np.rint(yy[None, :]-height*rows[:, None]).astype(int)
    pixels = lab[y.clip(0, lab.shape[0]-1), xx.clip(0, lab.shape[1]-1)].astype(float)
    refs = []
    for strip in (t < -.13, t > 1.13):
        valid = strip & (xx >= 0) & (xx < lab.shape[1])
        if valid.sum() >= 4:
            refs.append(np.median(pixels[:, valid], axis=1))
    if not refs:
        return [[0., 1.]]
    inner = (t >= .05) & (t <= .95)
    delta = np.minimum.reduce([np.linalg.norm(pixels[:, inner]-ref[:, None], axis=2) for ref in refs])
    different = delta > 44
    occupied = (different.mean(axis=0) >= .56) & (different[:7].mean(axis=0) >= .42)
    # Bridge a small gap between the legs of a silhouette.
    occupied = cv2.morphologyEx(occupied.astype('uint8')[None], cv2.MORPH_CLOSE,
                                np.ones((1, 6), np.uint8)).ravel()
    edges = np.flatnonzero(np.diff(np.r_[0, occupied, 0].astype(int)))
    ts = t[inner]
    return [[float(ts[a]-.005), float(ts[b-1]+.005)] for a,b in edges.reshape(-1,2) if b-a >= 6]


def free_goal_windows(goal, margin):
    """Unoccupied goal segments with room for the whole ball."""
    if not goal or not goal.get('shootable'):
        return []
    a, b = goal['ends']
    width = math.dist(a, b)
    if width <= 2*margin:
        return []
    pad = margin/width
    source = goal.get('opening_source', {})
    if 'occupied' not in source:
        return []
    free = []
    start = pad
    for lo, hi in sorted(source['occupied']):
        if start < lo-pad:
            free.append((start, lo-pad))
        start = max(start, hi+pad)
    if start < 1-pad:
        free.append((start, 1-pad))
    point = lambda t: [u+(v-u)*t for u,v in zip(a,b)]
    return [[point(left), point(right)] for left, right in free if (right-left)*width >= .035]


def corner_lane(ball, goal, margin):
    """Use the current shot if clear; otherwise aim toward a free corner."""
    windows = free_goal_windows(goal, margin)
    if not windows:
        return None
    a, b = goal['ends']
    width = math.dist(a, b)
    point = lambda t: [u+(v-u)*t for u,v in zip(a,b)]
    choices = []
    for window in windows:
        left, right = (math.dist(a, end)/width for end in window)
        if (right-left)*width >= .035 and shot_clear(ball, goal | {'ends': window}, 0.):
            return window
        for lo, hi in ((left,min(right,.40)), (max(left,.60),right)):
            if (hi-lo)*width < .035:
                continue
            target = point((lo+hi)/2)
            angle = math.atan2(target[1]-ball[1], target[0]-ball[0])
            choices.append((hi-lo, -abs(angle), [point(lo),point(hi)]))
    return max(choices, key=lambda c:(round(c[0],3),c[1]))[2] if choices else None


def field_obstacles(lab, turf, white, p, projector, q, geometry):
    """Unknown silhouettes supported by turf, in robot-relative metres."""
    import cv2
    import numpy as np
    from roki_ng.vision.colour_masks import mask
    contours, _ = cv2.findContours(turf.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return []
    hull = np.zeros(turf.shape, np.uint8)
    cv2.fillConvexPoly(hull, cv2.convexHull(max(contours, key=cv2.contourArea)), 1)
    unknown = hull & ~turf & ~white & (mask(lab, p, 'orange_ball') == 0)
    blobs = cv2.morphologyEx(unknown.astype(np.uint8), cv2.MORPH_CLOSE, np.ones((7,7), np.uint8))
    _, _, stats, _ = cv2.connectedComponentsWithStats(blobs)
    parts = []
    for x, y, w, h, area in stats[1:]:
        if area < 180 or w < 9 or h < .65*w or y+h >= turf.shape[0]-8:
            continue
        if turf[y+h:y+h+8, x:x+w].mean() < .55:
            continue
        try:
            base = plane_point(projector, (x+w/2, y+h), q, geometry)
            edge = plane_point(projector, (x, y+h), q, geometry)
        except ValueError:
            continue
        radius = math.dist(base, edge)
        if .035 <= radius <= .23 and .17 < math.hypot(*base) < 2:
            parts.append(dict(kind='obstacle', colour='unknown', points=[base], radius=radius,
                              area=int(area), rect=list(map(int,(x,y,x+w,y+h)))))
    return parts


def goal_box_support(ends, profiles, colour, paint, projector, q, geometry):
    """Look for the mapped front edge and a return of the penalty rectangle."""
    import numpy as np
    if not hasattr(projector, 'image_points'):
        return False
    a,b = np.asarray(ends)
    length = np.linalg.norm(b-a)
    if length < .2:
        return False
    across = (b-a)/length
    middle = (a+b)/2
    inward = np.array([-across[1],across[0]])
    if inward @ middle > 0:
        inward *= -1
    marks = [m for k,m in profiles.items() if k.startswith('field.mark.')
             and m['enabled'] and m['kind']=='line']
    for key in ('field.goal.0','field.goal.1'):
        goal = profiles.get(key)
        if not goal or (colour!='white' and goal['colour']!=colour):
            continue
        for front in marks:
            depth = abs(front['x']-goal['x'])
            if (not .08 < depth < .8 or abs(front['y']-goal['y'])>.07
                    or abs(math.cos(front['angle']))>.12 or front['size']<goal['width']*.9):
                continue
            def supported(start,end):
                body = start + np.linspace(.12,.88,15)[:,None]*(end-start)
                dx,dy = (body - [geometry['x'],geometry['y']]).T
                c,s = math.cos(geometry['pan']),math.sin(geometry['pan'])
                xy = np.column_stack((c*dx+s*dy,-s*dx+c*dy))*projector.height/geometry['height']
                xy[:,0] += getattr(projector,'camera_forward',0.)
                uv,seen = projector.image_points(xy,q)
                if seen.sum()<8:
                    return False
                u,v = np.rint(uv[seen]).astype(int).T
                on_line = paint[v.clip(0,649),u.clip(0,799)]
                # Bridge a central occlusion if both collinear ends and an
                # independent perpendicular edge agree with the field map.
                return (on_line.mean()>.65 or
                        (on_line.size>=12 and on_line[:4].all() and on_line[-4:].all()))
            half = front['size']/2
            near = middle+depth*inward
            if not supported(near-half*across,near+half*across):
                continue
            for sign in (-1,1):
                if any(abs(m['size']-depth)<.07 and abs(math.sin(m['angle']))<.12
                       and abs(m['x']-(front['x']+goal['x'])/2)<.07
                       and abs(abs(m['y']-goal['y'])-half)<.07 for m in marks):
                    if supported(middle+sign*half*across, near+sign*half*across):
                        return True
    return False


def goal_parts(image, p, projector, q, geometry, *, bearing_only=False):
    """Colour regions for steering; two image-side boundaries for a strike.

    Require a field-supported upright before reporting even a colour hint.
    A colour blob alone never becomes a shootable opening.
    """
    import cv2
    import numpy as np
    from roki_ng.vision.colour_masks import mask, shadow_mask
    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    boosted = cv2.cvtColor(cv2.convertScaleAbs(image, alpha=1.8), cv2.COLOR_BGR2LAB)
    green = shadow_mask(lab, boosted, p, 'green_field')
    joined = cv2.morphologyEx(green, cv2.MORPH_CLOSE, np.ones((31, 31), np.uint8))
    count, field_ids, field_stats, _ = cv2.connectedComponentsWithStats(joined)
    if count < 2:
        return []
    main = 1+int(np.argmax(field_stats[1:, cv2.CC_STAT_AREA]))
    if field_stats[main, cv2.CC_STAT_AREA] < green.size*.10:
        return []
    turf = (green != 0) & (field_ids == main)
    paint = shadow_mask(lab, boosted, p, 'white_marking')
    white_border = cv2.dilate(paint, np.ones((9, 9), np.uint8)) != 0
    parts = [] if bearing_only else field_obstacles(lab, turf, white_border, p, projector, q, geometry)
    for colour,recover in ((c,r) for c in ('blue','yellow','white') for r in (False,True)):
        if recover and (bearing_only or any(p['colour']==colour and p['kind']=='opening' for p in parts)):
            continue
        if colour == 'white' and bearing_only:
            continue
        raw = shadow_mask(lab, boosted, p, colour+'_posts') if recover else mask(lab,p,colour+'_posts')
        if recover and np.array_equal(raw,mask(lab,p,colour+'_posts')):
            continue
        binary = cv2.morphologyEx(raw, cv2.MORPH_CLOSE,
                                 np.ones((9, 9), np.uint8))
        n, labels, stats, _ = cv2.connectedComponentsWithStats(binary)
        feet = _supported_uprights(binary, labels, turf, stats)
        posts = []

        def opening(left, right, rect):
            if rect[0] <= 1 or rect[2] >= 799 or max(left[1], right[1]) >= 648:
                return  # Clipped side/base cannot establish the opening.
            try:
                ends = [plane_point(projector, pixel, q, geometry) for pixel in (left, right)]
            except ValueError:
                return
            bibs = keeper_span(image, left, right, rect) or []
            gaps = goal_line_gaps(paint,left,right)
            # A hole in the paint can also be a shadow. Only extend the
            # occupied lane when an independently detected robot overlaps it.
            for bib in bibs:
                for lo,hi in gaps:
                    if min(bib[1],hi)-max(bib[0],lo) >= .4*min(hi-lo,bib[1]-bib[0]):
                        bib[:] = [min(lo,bib[0]),max(hi,bib[1])]
            parts.append(dict(kind='opening', colour=colour, points=ends, rect=rect,
                              line_gaps=gaps, penalty_box=goal_box_support(ends,p,colour,white_border,projector,q,geometry),
                              keepers=bibs, occupied=sorted(bibs + occupied_goal_lanes(lab, left, right, rect)),
                              feet_px=[list(left), list(right)],
                              area=int(np.count_nonzero(binary[rect[1]:rect[3], rect[0]:rect[2]]))))

        if colour == 'white':
            # Paint can connect the entire goal, touchline and background into
            # one component. Track each shaft up from its own supported foot.
            for candidates in feet.values():
                for u,v in candidates:
                    top = v
                    centre = u
                    for row in range(v-4, max(0,v-320), -4):
                        lo,hi = max(0,centre-10), min(raw.shape[1],centre+11)
                        xs = np.flatnonzero(raw[row,lo:hi])
                        if len(xs)<3:
                            break
                        centre = round(.6*centre+.4*(lo+float(np.median(xs))))
                        top = row
                    if v-top < 42:
                        continue
                    # An upright breaks the turf skyline. Painted longitudinal
                    # lines have turf to both sides over most of their length.
                    band = turf[top:(v+top)//2,max(0,u-20):u+21]
                    if not band.size or band.mean() > .30:
                        continue
                    try:
                        point = plane_point(projector, (u,v), q, geometry)
                    except ValueError:
                        continue
                    rect = [max(0,min(u,centre)-7),top,min(raw.shape[1],max(u,centre)+8),v+1]
                    parts.append(dict(kind='post',colour=colour,points=[point],
                                      foot_px=[u,v],rect=rect,area=(v-top)*8))
                    posts.append((rect,(u,v)))
            posts.sort(key=lambda post: post[0][3]-post[0][1], reverse=True)
            for (a,af),(b,bf) in combinations(posts[:8],2):
                if af[0] > bf[0]:
                    a,af,b,bf = b,bf,a,af
                if bf[0]-af[0] >= 30 and min(a[3]-a[1],b[3]-b[1]) >= .5*max(a[3]-a[1],b[3]-b[1]):
                    opening(af,bf,[a[0],min(a[1],b[1]),b[2],max(a[3],b[3])])
            continue

        for i in sorted((i for i in feet if i), key=lambda j: int(stats[j, 4]), reverse=True)[:6]:
            x, y, w, h, area = map(int, stats[i])
            if area < (30 if bearing_only else 120) or h < (4 if bearing_only else 12) or w < (4 if bearing_only else 6):
                continue
            component_pixels = (labels[y:y+h, x:x+w] == i) & (raw[y:y+h, x:x+w] != 0)
            # Colour fringing along white pitch tubes must not become a goal.
            if (component_pixels.any() and
                    white_border[y:y+h, x:x+w][component_pixels].mean() > .65):
                continue
            rect = [x, y, x+w, y+h]
            try:
                ray = projector._pixel_ray((x+w/2, y+h/2), q)
                bearing = wrap(math.atan2(float(ray[1]), float(ray[0]))+geometry['pan'])
                edge_angles = []
                for u in ((x+w/2, x+w/2) if bearing_only else (x, x+w-1)):
                    ray = projector._pixel_ray((u, y+h/2), q)
                    edge_angles.append(wrap(math.atan2(float(ray[1]), float(ray[0]))+geometry['pan']))
            except ValueError:
                # Fisheye frames include pixels outside the calibrated view.
                # Reject this region, not the complete ball/goal worker.
                continue
            for u, v in feet[i]:
                try:
                    point = plane_point(projector, (u, v), q, geometry)
                except ValueError:
                    continue
                if not bearing_only:
                    parts.append(dict(kind='post', colour=colour, points=[point],
                                      rect=rect, foot_px=[u,v], area=area))
            parts.append(dict(kind='colour', colour=colour, points=[[2*math.cos(bearing), 2*math.sin(bearing)]],
                              rect=rect, bearings=edge_angles, area=area))
            if bearing_only:
                continue  # Keeper steering only: never declare a shootable opening.
            component = labels[y:y+h, x:x+w] == i
            # Follow the two outside edges row-by-row, even on curved/leaning
            # tubes. Middle rows exclude both upper AND lower crossbars.
            rows = []
            for row in range(int(h*.2), max(int(h*.2)+1, int(h*.8))):
                xs = np.flatnonzero(component[row])
                if len(xs) < 2:
                    continue
                lo, hi = int(xs[0]), int(xs[-1])
                if hi-lo >= max(24, w*.4) and component[row, lo:hi+1].mean() < .55:
                    rows.append((row, lo, hi))
            if h >= 30 and len(rows) >= max(10, int(h*.25)):
                rows = np.asarray(rows, float)
                # Extrapolate only within this component, to the lower base.
                left = float(np.polyval(np.polyfit(rows[:, 0], rows[:, 1], 1), h-1))
                right = float(np.polyval(np.polyfit(rows[:, 0], rows[:, 2], 1), h-1))
                left, right = max(0., min(w-1., left)), max(0., min(w-1., right))
                # Perspective puts the two post bases at different image rows.
                # Match each supported base against its own fitted upright.
                bases = []
                supported_count = 0
                for column in (1, 2):
                    fit = np.polyfit(rows[:, 0], rows[:, column], 1)
                    candidates = [(u, v) for u, v in feet[i]
                                  if v >= y+h*.55
                                  and abs(u-(x+float(np.polyval(fit, v-y)))) <= max(10, w*.10)]
                    if candidates:
                        bases.append(max(candidates, key=lambda point: point[1]))
                        supported_count += 1
                        continue
                    # A front post may end on the painted goal line. Follow
                    # its fitted edge down to the last coloured pixel and
                    # require both white paint and adjacent turf at that foot.
                    for v in range(h-1, int(.55*h), -1):
                        u = round(float(np.polyval(fit, v)))
                        lo, hi = max(0, u-6), min(w, u+7)
                        if lo >= hi or not component[v, lo:hi].any():
                            continue
                        px, py = x+lo+int(np.median(np.flatnonzero(component[v, lo:hi]))), y+v
                        strip = slice(max(0,px-10), px+11)
                        if (white_border[py+1:py+10, strip].any() and
                                turf[py+1:py+25, strip].any()):
                            bases.append((px, py))
                        break
                if supported_count and len(bases) == 2 and bases[1][0]-bases[0][0] >= 24:
                    opening(bases[0], bases[1], rect)
                continue
            if h >= 30 and h >= 1.3*w:
                xs = np.flatnonzero(np.any(component[max(0, h-5):], axis=0))
                if len(xs):
                    posts.append((rect, (x+float(np.median(xs)), y+h-1)))
        # Separate colour components can be the two uprights without a bar.
        for (a, af), (b, bf) in combinations(posts, 2):
            if a[0] > b[0]:
                a, af, b, bf = b, bf, a, af
            overlap = min(a[3], b[3])-max(a[1], b[1])
            if b[0]-a[2] >= 20 and overlap >= .4*min(a[3]-a[1], b[3]-b[1]):
                opening(af, bf, [a[0], min(a[1], b[1]), b[2], max(a[3], b[3])])
    # Individual feet are needed only for a partially observed frame.
    complete = {part['colour'] for part in parts if part['kind']=='opening' and part['colour']!='white'}
    unique = {}
    for part in parts:
        if part['kind']=='post' and part['colour'] in complete:
            continue
        key = (part['colour'],part['kind'],tuple(part.get('foot_px',part['rect'])))
        unique.setdefault(key,part)
    return list(unique.values())


def assemble_goals(parts, specs):
    """Select an opening, or a non-shootable colour hint for steering."""
    result = []
    for spec in specs:
        candidates = [part for part in parts if part['colour'] == spec['colour'] and part['kind'] == 'opening']
        hints = [part for part in parts if part['colour'] == spec['colour'] and part['kind'] == 'colour']
        bearings = [part['bearings'] for part in hints]
        corroborated = [part for part in candidates if part.get('penalty_box')]
        if corroborated:
            candidates = corroborated
        if candidates:
            ends = candidates[0]['points']
            centre = [(a+b)/2 for a, b in zip(*ends)]
            ambiguous = any(math.dist(centre, [(a+b)/2 for a, b in zip(*other['points'])]) > .5
                            for other in candidates[1:])
            if not ambiguous:
                result.append(dict(colour=spec['colour'], ends=ends, elements='opening',
                                   shootable=True, bearings=bearings, colour_sources=hints,
                                   opening_source=candidates[0]))
                continue
        if hints:
            hint = max(hints, key=lambda part: part['area'])
            result.append(dict(colour=spec['colour'], ends=hint['points']*2, elements='colour only',
                               shootable=False, bearings=bearings, colour_sources=hints))
    return result


def own_colour_ahead(goal):
    """A visible own-colour region can veto a kick without a complete goal."""
    for left, right in goal.get('bearings', ()):
        half = abs(wrap(right-left))/2
        centre = wrap(left+wrap(right-left)/2)
        if abs(centre) <= half+math.radians(10):
            return True
    return False


def own_goal_blocks(ball, goal, radius):
    """Explain the existing veto, preserving its exact colour/opening rules."""
    if not goal:
        return []
    blocks = []
    def detail(reason, source, **extra):
        return dict(reason=reason, colour=goal['colour'],
                    **{key: source.get(key) for key in ('frame_sequence', 'sensor_timestamp_ns',
                       'pan', 'tilt', 'rect', 'area', 'parameter_revision')}, **extra)
    for index, (left, right) in enumerate(goal.get('bearings', ())):
        centre = wrap(left+wrap(right-left)/2)
        if abs(centre) <= abs(wrap(right-left))/2+math.radians(10):
            sources = goal.get('colour_sources', [])
            source = sources[index] if index < len(sources) else {}
            blocks.append(detail('colour_ahead', source, bearing_deg=math.degrees(centre),
                                 edges_deg=[math.degrees(left), math.degrees(right)]))
    if ball is not None and shot_clear(ball, goal, -radius):
        blocks.append(detail('shot_intersects_opening', goal.get('opening_source', {}),
                             ends=goal['ends']))
    return blocks


def shot_clear(ball, goal, margin, heading=0.):
    """Ray from ball through the interior of a goal segment (metres)."""
    if not goal.get('shootable', False):
        return False
    a, b = goal['ends']
    d = [math.cos(heading), math.sin(heading)]
    v = [b[0]-a[0], b[1]-a[1]]
    w = [a[0]-ball[0], a[1]-ball[1]]
    cross = lambda x, y: x[0]*y[1]-x[1]*y[0]
    det = cross(d, v)
    length = math.hypot(*v)
    if abs(det) < 1e-6 or length <= 2*margin:
        return False
    along, side = cross(w, v)/det, cross(w, d)/det
    return along > 0 and margin/length <= side <= 1-margin/length


class SceneTracker:
    def __init__(self):
        self.previous = None
        self.current = None

    def update(self, scene):
        previous = self.previous
        self.previous = scene
        valid = previous and previous['pose_id'] == scene['pose_id'] and scene['sequence'] > previous['sequence']
        old_ball = previous.get('ball') if valid else None
        ball = scene.get('ball')
        confirmed_ball = ball if ball and old_ball and math.dist(ball, old_ball) <= .06 else None
        parts = []
        for part in scene['parts']:
            if valid and any(old['kind'] == part['kind'] and old['colour'] == part['colour']
                             and len(old['points']) == len(part['points'])
                             and max(math.dist(a, b) for a, b in zip(old['points'], part['points'])) <= .25
                             for old in previous['parts']):
                parts.append(part)
        foot = scene.get('foot')
        prior = previous.get('foot') if valid else None
        if not (confirmed_ball and foot and prior and foot['leg'] == prior['leg']
                and foot['aligned'] == prior['aligned']
                and abs(foot['gap_ratio']-prior['gap_ratio']) < .15):
            foot = None
        self.current = scene | {'frame_obstacles': [
            (part['points'][0], part['radius']) for part in scene['parts'] if part['kind'] == 'obstacle'
        ], 'foot': foot, 'ball': confirmed_ball, 'parts': parts, 'confirmed': bool(valid)}
        return self.current


def render_scene(image, scene, note, specs=(), own_colour=None, decision=None):
    import cv2
    import numpy as np
    output = image.copy()
    if scene:
        foot = scene.get('foot')
        if foot:
            for leg, (x1, y, x2) in zip(('left', 'right'), foot['tips']):
                cv2.line(output, (x1, y), (x2, y), (255, 255, 0), 3)
                if leg == foot['leg']:
                    cv2.putText(output, leg, (x1, y+17), cv2.FONT_HERSHEY_SIMPLEX, .5, (255,255,0), 1)
            x1,y1,x2,y2 = foot['ball_rect']
            cv2.rectangle(output, (x1,y1), (x2,y2), (0, 160, 255), 2)
        for part in scene['parts']:
            x, y, x2, y2 = part['rect']
            colour = (0, 255, 0) if part['kind'] == 'opening' else (0, 180, 255)
            if part['kind'] == 'opening' and 'feet_px' in part:
                left, right = np.asarray(part['feet_px'])
                for lo, hi in part.get('line_gaps', []):
                    for t in np.arange(lo,hi,.025):
                        start,end = [tuple(np.rint(left+(right-left)*u).astype(int))
                                     for u in (t,min(hi,t+.012))]
                        cv2.line(output,start,end,(255,255,0),2)
                if part.get('penalty_box'):
                    cv2.putText(output,'box',(x,y+20),cv2.FONT_HERSHEY_SIMPLEX,.55,(0,240,120),2)
                for lo, hi in part['occupied']:
                    start, end = [tuple(np.rint(left+(right-left)*t).astype(int)) for t in (lo, hi)]
                    cv2.line(output, start, end, (0, 0, 255), 6)
            cv2.rectangle(output, (x, y), (x2, y2), colour, 2)
            cv2.putText(output, part['colour']+' '+part['kind'], (x, max(20, y-5)),
                        cv2.FONT_HERSHEY_SIMPLEX, .6, colour, 2)
    if scene:
        inset = np.zeros((260, 240, 3), np.uint8)
        point = lambda xy: (int(120-xy[1]*70), int(240-xy[0]*70))
        cv2.circle(inset, point([0, 0]), 5, (255, 255, 255), -1)
        for goal in assemble_goals(scene['parts'], specs):
            if goal['shootable']:
                cv2.line(inset, point(goal['ends'][0]), point(goal['ends'][1]), (255, 255, 0), 4)
                if scene.get('ball') and own_colour and goal['colour'] != own_colour:
                    lane = corner_lane(scene['ball'], goal, scene['ball_radius']+.05)
                    if lane:
                        target = [(a+b)/2 for a,b in zip(*lane)]
                        cv2.arrowedLine(inset, point(scene['ball']), point(target), (255, 0, 255), 2)
                        feet = np.asarray(goal['opening_source']['feet_px'])
                        t = math.dist(goal['ends'][0], target)/math.dist(*goal['ends'])
                        aim = tuple(np.rint(feet[0]+t*(feet[1]-feet[0])).astype(int))
                        cv2.drawMarker(output, aim, (255, 0, 255), cv2.MARKER_CROSS, 18, 2)
            cv2.putText(inset, goal['colour']+' '+goal['elements'], (4, 20 if goal['colour'] == 'blue' else 40),
                        cv2.FONT_HERSHEY_SIMPLEX, .4, (255, 255, 255), 1)
        if scene.get('ball'):
            bx, by = scene['ball']
            cv2.circle(inset, point([bx, by]), 4, (0, 140, 255), -1)
            cv2.arrowedLine(inset, point([bx, by]), point([bx+2, by]), (0, 255, 0), 2)
        output[20:280, 550:790] = inset
    cv2.rectangle(output, (0, 540), (799, 649), (0, 0, 0), -1)
    ball = scene.get('ball') if scene else None
    text = f'BALL X={ball[0]*1000:.0f} Y={ball[1]*1000:+.0f} D={math.hypot(*ball)*1000:.0f} mm' if ball else 'BALL: no confirmed support-pose observation'
    cv2.putText(output, text, (8, 570), cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 255, 0), 2)
    height = f"H={scene['height_m']*1000:.0f} mm" if scene else 'geometry inactive'
    cv2.putText(output, height+' | '+note[:75], (8, 601), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1)
    cv2.putText(output, note[75:150], (8, 629), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1)
    if scene and own_colour:
        own = next((goal for goal in assemble_goals(scene['parts'], specs)
                    if goal['colour'] == own_colour), None)
        for block in own_goal_blocks(scene.get('ball'), own, scene.get('ball_radius', .045)):
            rect = block.get('rect')
            if rect:
                x, y, x2, y2 = map(int, rect)
                cv2.rectangle(output, (x, y), (x2, y2), (0, 0, 255), 3)
                cv2.putText(output, 'OWN GOAL BLOCK: '+block['reason'], (max(0,x), max(22,y-6)),
                            cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 0, 255), 2)
        cv2.putText(output, f"Frame check #{scene['sequence']}; actual decision sources in log",
                    (8, 25), cv2.FONT_HERSHEY_SIMPLEX, .5, (255,255,255), 1)
    if decision and decision.get('blocks') and 0 <= time.monotonic()-decision['received'] <= 3:
        block = decision['blocks'][0]
        cv2.rectangle(output, (0, 606), (799, 649), (0, 0, 0), -1)
        label = (f"LAST OWN GOAL BLOCK: {block['reason']} source #{block.get('frame_sequence')} "
                 f"pan={block.get('pan')} tilt={block.get('tilt')}")
        cv2.putText(output, label, (8, 631), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 0, 255), 1)
    return output
