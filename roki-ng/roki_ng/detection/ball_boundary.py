"""Check which side of a visible outer painted line contains the ball centre."""
import cv2
import numpy as np

# Local floor patch in metres, centred on the projected ball.
RESOLUTION = .02
axis = np.arange(-48, 49) * RESOLUTION
GRID = np.stack(np.meshgrid(axis, axis), axis=-1).reshape(-1, 2)


def outside_paint(green, white, valid, geometry):
    """Masks are metric, with the ball at the centre of the patch."""
    margins = [(geometry['carpet_'+k]-geometry[k])/2 for k in ('width', 'length')]
    paint = geometry['paint_width']
    markings = np.uint8((white != 0) & valid)*255
    lines = cv2.HoughLinesP(markings, 1, np.pi/180, 16, minLineLength=22, maxLineGap=3)
    if lines is None:
        return False
    centre = (np.array(valid.shape[::-1])-1)/2

    def sample(points):
        idx = np.rint(points).astype(int)
        x, y = idx[:, 0], idx[:, 1]
        ok = (x >= 0) & (y >= 0) & (x < valid.shape[1]) & (y < valid.shape[0])
        x, y = x.clip(0, valid.shape[1]-1), y.clip(0, valid.shape[0]-1)
        return valid[y,x] & ok, green[y,x] != 0

    for x,y,X,Y in lines.reshape(-1,4):
        delta = np.array([X-x,Y-y], float)
        length = np.linalg.norm(delta)
        tangent = delta/length
        normal = np.array([-tangent[1], tangent[0]])
        midpoint = np.array([(x+X)/2, (y+Y)/2])
        displacement = centre-midpoint
        distance = displacement @ normal
        # Preserve balls on the paint and avoid extrapolating beyond a corner.
        if abs(distance)*RESOLUTION <= paint+.015 or abs(displacement@tangent)>length/2+min(.20/RESOLUTION, .3*length):
            continue
        outward = normal*np.sign(distance)
        samples = midpoint + np.linspace(-.43,.43,11)[:,None]*delta
        for margin in set(margins):
            if margin <= paint or abs(distance)*RESOLUTION > margin+.25:
                continue
            votes = np.ones(len(samples), bool)
            # Turf on both sides of paint, then only on the field side beyond
            # the configured apron. Unknown/off-camera pixels cannot vote.
            for offset, turf in ((-.09,True), (margin*.4,True),
                                 (-.17,True), (margin+.08,False), (margin+.16,False)):
                seen, is_green = sample(samples + outward*(offset/RESOLUTION))
                votes &= seen & (is_green == turf)
            if votes.sum() >= 6:
                return True
    return False


def outside_field(point, projector, quaternion, green, white, geometry, height):
    # image_points uses the projector's nominal ground height.
    pixels, visible = projector.image_points((GRID+point)*(projector.height/height), quaternion)
    ix = np.rint(pixels[:,0]).astype(int).clip(0,green.shape[1]-1)
    iy = np.rint(pixels[:,1]).astype(int).clip(0,green.shape[0]-1)
    shape = (len(axis),len(axis))
    return outside_paint(green[iy,ix].reshape(shape), white[iy,ix].reshape(shape),
                         visible.reshape(shape), geometry)
