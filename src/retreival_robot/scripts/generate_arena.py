#!/usr/bin/env python3
"""Single source of truth: emit SDF world, landmark JSON and aligned occupancy PGM/YAML."""
import json, math, pathlib, xml.etree.ElementTree as ET
import numpy as np

OUT = pathlib.Path(__file__).resolve().parents[1]
W, H, RES = 14.0, 12.0, 0.05
XMIN, YMIN = -7., -6.
# Asymmetric positions reduce ambiguity when all observed cylinders look identical.
LANDMARKS = [
 ('C01',-5.4,-4.4,'red'), ('C02',-3.1,-4.0,'green'), ('C03',-0.9,-4.9,'blue'),
 ('C04',2.0,-3.9,'yellow'), ('C05',5.3,-4.5,'magenta'),
 ('C06',-5.6,-1.5,'cyan'), ('C07',-2.6,-1.2,'yellow'), ('C08',1.3,-1.6,'red'),
 ('C09',4.5,-1.0,'blue'), ('C10',-5.0,2.1,'magenta'),
 ('C11',-1.9,1.6,'cyan'), ('C12',1.0,2.6,'green'), ('C13',4.9,1.8,'red'),
 ('C14',-4.7,4.4,'blue'), ('C15',-0.5,4.7,'magenta'), ('C16',3.9,4.2,'yellow')
]
COLORS={'red':(1,0.05,0.05),'green':(0.05,1,0.05),'blue':(0.1,0.2,1),
 'yellow':(1,0.9,0.05),'magenta':(1,0.05,1),'cyan':(0.05,1,1)}
RADIUS=0.06
WALL_THICK=.15

def elem(parent,tag,attrs=None,text=None):
    x=ET.SubElement(parent,tag,attrs or {})
    if text is not None:x.text=str(text)
    return x

def box_model(world,name,x,y,z,sx,sy,sz):
    model=elem(world,'model',{'name':name});elem(model,'static',text='true')
    link=elem(model,'link',{'name':'link'});elem(link,'pose',text=f'{x} {y} {z} 0 0 0')
    for kind in ('collision','visual'):
        item=elem(link,kind,{'name':kind});geo=elem(item,'geometry')
        elem(elem(geo,'box'),'size',text=f'{sx} {sy} {sz}')

def main():
    (OUT/'worlds').mkdir(exist_ok=True);(OUT/'maps').mkdir(exist_ok=True)
    sdf=elem(ET.Element('sdf',{'version':'1.9'}),'world',{'name':'retrieval_arena'})
    elem(sdf,'gravity',text='0 0 -9.8')
    elem(sdf,'plugin',{'filename':'gz-sim-physics-system','name':'gz::sim::systems::Physics'})
    elem(sdf,'plugin',{'filename':'gz-sim-user-commands-system','name':'gz::sim::systems::UserCommands'})
    elem(sdf,'plugin',{'filename':'gz-sim-scene-broadcaster-system','name':'gz::sim::systems::SceneBroadcaster'})
    elem(sdf,'plugin',{'filename':'gz-sim-sensors-system','name':'gz::sim::systems::Sensors'})
    elem(elem(sdf,'scene'),'ambient',text='0.75 0.75 0.75 1')
    light=elem(sdf,'light',{'name':'sun','type':'directional'})
    elem(light,'pose',text='0 0 10 0 0 0')
    elem(light,'direction',text='-0.3 -0.4 -1')
    elem(light,'diffuse',text='0.9 0.9 0.9 1')
    elem(light,'specular',text='0.2 0.2 0.2 1')
    box_model(sdf,'floor',0,0,-0.1,W,H,0.2)
    box_model(sdf,'wall_north',0,6,0.45,W,.15,.9)
    box_model(sdf,'wall_south',0,-6,0.45,W,.15,.9)
    box_model(sdf,'wall_east',7,0,0.45,.15,H,.9)
    box_model(sdf,'wall_west',-7,0,0.45,.15,H,.9)
    for name,x,y,color in LANDMARKS:
        model=elem(sdf,'model',{'name':f'landmark_{name.lower()}'})
        elem(model,'static',text='true');link=elem(model,'link',{'name':'link'})
        elem(link,'pose',text=f'{x} {y} 0.30 0 0 0')
        for kind in ('collision','visual'):
            item=elem(link,kind,{'name':kind})
            cylinder=elem(elem(item,'geometry'),'cylinder')
            elem(cylinder,'radius',text=RADIUS);elem(cylinder,'length',text=0.6)
            if kind=='visual':
                material=elem(item,'material')
                rgb=COLORS[color];rgba=' '.join(map(str,(*rgb,1)))
                elem(material,'ambient',text=rgba);elem(material,'diffuse',text=rgba)
    ET.indent(sdf.getroottree() if hasattr(sdf,'getroottree') else sdf)
    # SDF world is written with correct SDF root.
    tree=sdf
    worldroot=ET.Element('sdf',{'version':'1.9'});worldroot.append(tree)
    ET.indent(worldroot)
    ET.ElementTree(worldroot).write(OUT/'worlds'/'retrieval_landmarks.sdf',encoding='unicode',xml_declaration=True)
    data={'frame_id':'map','world':'retrieval_arena','radius_m':RADIUS,
          'landmarks':[{'id':name,'x':x,'y':y,'z_base':0.,'color':color,'radius':RADIUS}
                       for name,x,y,color in LANDMARKS]}
    (OUT/'maps'/'landmarks.json').write_text(json.dumps(data,indent=2)+'\n')
    nx,ny=int(round(W/RES)),int(round(H/RES))
    # Row zero of PGM is top (highest world y); origin in YAML is bottom-left.
    yy,xx=np.mgrid[0:ny,0:nx]
    wx=XMIN+(xx+.5)*RES; wy=YMIN+(ny-yy-.5)*RES
    occupied=((np.abs(wx)>=W/2-WALL_THICK/2) | (np.abs(wy)>=H/2-WALL_THICK/2))
    for _,x,y,_ in LANDMARKS:
        occupied|=(wx-x)**2+(wy-y)**2 <= RADIUS**2
    pixels=np.where(occupied,0,254).astype(np.uint8)
    with open(OUT/'maps'/'retrieval_landmarks.pgm','wb') as f:
        f.write(f'P5\n{nx} {ny}\n255\n'.encode());f.write(pixels.tobytes())
    (OUT/'maps'/'retrieval_landmarks.yaml').write_text(
        f'image: retrieval_landmarks.pgm\nresolution: {RES}\norigin: [{XMIN}, {YMIN}, 0.0]\n'
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\nmode: trinary\n')
    print('Generated SDF, JSON, occupancy map, map YAML from one landmark table')

if __name__=='__main__': main()
