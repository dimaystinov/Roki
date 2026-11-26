"""Find tested locations that also succeeded at the eight adjacent 5 mm offsets."""
from collections import defaultdict


def target(rows, placement, spacing=5):
    outcomes = defaultdict(list)
    for row in rows:
        outcomes[tuple(row["placement"])].append(row["usable"])
    passed = {p for p, outcomes_at_point in outcomes.items() if all(outcomes_at_point)}
    centres = {p for p in passed if all((p[0]+dx, p[1]+dy) in passed
               for dx in (-spacing, 0, spacing) for dy in (-spacing, 0, spacing))}
    if not centres:
        return None
    x,y = placement
    front,side = min(centres, key=lambda p: ((p[0]-x)**2+(p[1]-y)**2,p))
    return {"front_mm":front,"side_mm":side,"spacing_mm":spacing}
