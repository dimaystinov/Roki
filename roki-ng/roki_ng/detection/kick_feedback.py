"""Compare ball positions in two downward views using the nearby pitch markings."""
import base64
import math

def kick_view(image, params, rect):
    import cv2
    import numpy as np
    from roki_ng.vision.colour_masks import mask, bounds
    small = cv2.resize(image, (200, 162), interpolation=cv2.INTER_AREA)
    lab = cv2.cvtColor(small, cv2.COLOR_BGR2LAB)
    turf = mask(lab, params, 'green_field')
    low, high = bounds(params, 'white_marking')
    paint = cv2.inRange(lab,(max(0,low[0]-48),*low[1:]),high)
    nearby = cv2.dilate(turf, np.ones((9,9),np.uint8))
    paint &= nearby
    x,y,r,b = rect
    centre = [(x+r)/8, (y+b)*81/650]
    diameter = max(r-x,b-y)/4
    if diameter < 3:
        return None
    # Discard spectators, toes, and any bright highlight on the ball.
    paint[:12] = paint[122:] = 0
    cv2.circle(paint,tuple(round(v) for v in centre),round(diameter*.46),0,-1)
    return dict(paint=base64.b64encode(cv2.imencode('.png',paint)[1]).decode(),
                centre=centre,diameter=diameter)

def kick_result(before, after):
    """Return displacement in ball diameters, or None for an ambiguous comparison."""
    import cv2
    import numpy as np
    if not before or not after:
        return None
    def marks(view):
        raw=np.frombuffer(base64.b64decode(view['paint']),np.uint8)
        return cv2.imdecode(raw,cv2.IMREAD_GRAYSCALE)
    a,b=marks(before),marks(after)
    if min(cv2.countNonZero(a),cv2.countNonZero(b)) < 120:
        return None
    try:
        score,warp=cv2.findTransformECC(a,b,np.eye(2,3,dtype=np.float32),cv2.MOTION_EUCLIDEAN,
                                        (cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,50,.0001))
    except cv2.error:
        return None
    if score < .77 or abs(math.atan2(warp[1,0],warp[0,0])) > .30:
        return None
    centre=warp @ np.array([*before['centre'],1.])
    u,v=np.rint(centre).astype(int)
    if not (0<=u<b.shape[1] and 0<=v<b.shape[0]):
        return None
    # Require locally matching paint near the predicted ball as well as global agreement.
    aligned=cv2.warpAffine(a,warp,(b.shape[1],b.shape[0]))
    roi=np.s_[max(0,v-35):v+36,max(0,u-35):u+36]
    local=aligned[roi]>0
    if local.sum()<30 or (cv2.dilate(b,np.ones((5,5),np.uint8))[roi][local]>0).mean()<.7:
        return None
    return float(np.linalg.norm(centre-after['centre'])/before['diameter'])
