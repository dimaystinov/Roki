"""Shared LAB units and thresholds for tuning and localization."""
import math


def bounds(parameters,profile):
    prefix=f'vision.{profile}.'
    return ((int(parameters[prefix+'l_min']*2.55),parameters[prefix+'a_min']+128,parameters[prefix+'b_min']+128),
            (math.ceil(parameters[prefix+'l_max']*2.55),parameters[prefix+'a_max']+128,parameters[prefix+'b_max']+128))


def mask(lab,parameters,profile):
    import cv2
    return cv2.inRange(lab,*bounds(parameters,profile))


def shadow_mask(lab, brighter, parameters, profile):
    """Recover calibrated colours in dim pixels without widening bright masks."""
    import cv2
    normal = mask(lab, parameters, profile)
    lower, upper = bounds(parameters, profile)
    if profile in ('white_marking', 'white_posts'):
        recovered = cv2.inRange(lab, (int(lower[0]*.65), *lower[1:]), (lower[0], *upper[1:]))
    else:
        dark = cv2.inRange(lab[:, :, 0], 8, max(8, lower[0]-1))
        recovered = cv2.bitwise_and(dark, mask(brighter, parameters, profile))
    return cv2.bitwise_or(normal, recovered)
