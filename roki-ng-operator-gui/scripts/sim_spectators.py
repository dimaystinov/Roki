"""Continuous pedestrian traffic outside the pitch for camera stress tests."""
import math
import xml.etree.ElementTree as ET


SHIRTS = ('0.02 0.08 0.9', '1 0.8 0.02', '0.9 0.03 0.05',
          '0.08 0.65 0.1', '0.93 0.93 0.93', '0.95 0.25 0.03', '0.08 0.72 0.78')
CHILD = len(SHIRTS)-1


def add_spectators(world):
    for kind, pos, size, rgb in (
        ('ellipsoid', '.42 1.64 .12', '.22 .105 .12', '1 .27 .01'),
        ('box', '-2.58 -.84 .34', '.03 .24 .34', '1 .88 .02'),
        ('sphere', '-.56 -1.41 .045', '.045', '1 .26 .01'),
    ):
        ET.SubElement(world, 'geom', type=kind, pos=pos, size=size,
                      rgba=rgb+' 1', contype='0', conaffinity='0')

    for i, shirt in enumerate(SHIRTS):
        scale = .61 if i == CHILD else 1.
        def shape(body, kind, pos, size, colour, name):
            scaled = lambda values: ' '.join(str(v*scale) for v in values)
            ET.SubElement(body, 'geom', name=name, type=kind, pos=scaled(pos), size=scaled(size),
                          rgba=colour+' 1', contype='0', conaffinity='0')
        person = ET.SubElement(world, 'body', name=f'spectator_{i}', mocap='true', pos='3 3 0')
        shape(person,'ellipsoid',(0,0,1.14),(.16,.23,.34),shirt,f'shirt_{i}')
        shape(person,'sphere',(0,0,1.60),(.13,),'.65 .43 .31',f'face_{i}')
        for side in (-1,1):
            shape(person,'capsule',(0,side*.255,1.12),(.052,.19),shirt,f'arm_{i}_{side}')
            # Rotate the leg about the hip. The shoe follows on the same body.
            leg = ET.SubElement(world,'body',name=f'walker_leg_{i}_{side}',mocap='true',pos='3 3 0')
            shape(leg,'capsule',(0,0,-.39),(.073,.38),'.07 .08 .11',f'trouser_{i}_{side}')
            shape(leg,'ellipsoid',(.042,0,-.818),(.145,.063,.062),'1 .24 .01',f'shoe_{i}_{side}')


def spectator_ids(model):
    ball = model.geom('ball').id
    ids = []
    for i in range(len(SHIRTS)):
        torso = model.body(f'spectator_{i}').mocapid.item()
        legs = tuple(model.body(f'walker_leg_{i}_{s}').mocapid.item() for s in (-1,1))
        ids.append((torso,legs))
        if i == CHILD:
            for side in (-1,1):
                shoe = model.geom(f'shoe_{i}_{side}').id
                model.geom_matid[shoe] = model.geom_matid[ball]
                model.geom_rgba[shoe] = model.geom_rgba[ball]
    return ids


def move_spectators(data, ids, layout, t):
    for i, (torso,legs) in enumerate(ids):
        phase = t*(.48+.035*i) + 2.1*i
        sine, cosine = math.sin(phase), math.cos(phase)
        side = 1 if i%2 else -1
        if i < 4:
            x = side*(abs(layout['blue_goal_x_m'])+.80+.23*(i//2)+.11*cosine)
            y = 1.27*sine
            dx,dy = -side*.11*sine,1.27*cosine
        else:
            # The child passes close to the touchline, including the green apron.
            distance = layout['field_width_m']/2+(.02 if i==CHILD else .40)
            x = 1.60*sine
            y = side*(distance+.08*cosine)
            dx,dy = 1.60*cosine,-side*.08*sine
        yaw = math.atan2(dy,dx)
        cy,sy = math.cos(yaw/2),math.sin(yaw/2)
        stride = t*(5.5+.13*i)+i
        scale = .61 if i==CHILD else 1.
        bob = .012*scale*abs(math.sin(stride))
        data.mocap_pos[torso] = (x,y,bob)
        data.mocap_quat[torso] = (cy,0,0,sy)
        for sign,leg in zip((-1,1),legs):
            pitch = sign*.30*math.sin(stride)
            cp,sp = math.cos(pitch/2),math.sin(pitch/2)
            lateral = sign*.105*scale
            data.mocap_pos[leg] = (x-math.sin(yaw)*lateral,y+math.cos(yaw)*lateral,.88*scale+bob)
            data.mocap_quat[leg] = (cy*cp,-sy*sp,cy*sp,sy*cp)
