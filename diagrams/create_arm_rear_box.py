"""Author native draw.io XML and a matching PNG preview; no hardware access."""
from pathlib import Path
from xml.etree import ElementTree as ET
from PIL import Image, ImageDraw, ImageFont, PngImagePlugin
import math

OUT = Path(__file__).resolve().parent
W, H, SCALE = 1400, 830, 2
canvas = Image.new('RGB', (W * SCALE, H * SCALE), '#f5f7fb')
draw = ImageDraw.Draw(canvas)
graph = ET.Element('mxGraphModel', adaptiveColors='auto', pageWidth=str(W), pageHeight=str(H))
root = ET.SubElement(graph, 'root')
ET.SubElement(root, 'mxCell', id='0')
ET.SubElement(root, 'mxCell', id='1', parent='0')
counter = 1
font_path = Path(r'C:\Windows\Fonts\msyh.ttc')
fonts = {}

def font(size):
    if size not in fonts:
        fonts[size] = ImageFont.truetype(str(font_path), round(size * SCALE))
    return fonts[size]

def cell(value, style, vertex=False):
    global counter
    counter += 1
    attrs = dict(id=str(counter), value=value, style=style+';html=1;', parent='1')
    attrs['vertex' if vertex else 'edge'] = '1'
    return ET.SubElement(root, 'mxCell', attrs)

def rect(x, y, w, h, fill='#ffffff', stroke='#dce3ec', radius=0):
    c = cell('', f'rounded={int(radius>0)};fillColor={fill};strokeColor={stroke};strokeWidth=2;', True)
    ET.SubElement(c, 'mxGeometry', x=str(x), y=str(y), width=str(w), height=str(h), **{'as':'geometry'})
    coords=tuple(round(v*SCALE) for v in (x,y,x+w,y+h))
    if radius:
        draw.rounded_rectangle(coords, radius=radius*SCALE, fill=fill, outline=stroke, width=2*SCALE)
    else:
        draw.rectangle(coords, fill=fill, outline=stroke, width=2*SCALE)

def text(x,y,w,h,value,size=20,color='#334155',align='left',bold=False):
    c = cell(value, f'text;whiteSpace=wrap;align={align};verticalAlign=middle;fontSize={size};fontFamily=Microsoft YaHei;fontColor={color};fontStyle={int(bold)};strokeColor=none;fillColor=none;', True)
    ET.SubElement(c,'mxGeometry',x=str(x),y=str(y),width=str(w),height=str(h),**{'as':'geometry'})
    lines=value.split('\n')
    line_h=size*1.4
    for i, line in enumerate(lines):
        bounds=draw.textbbox((0,0),line,font=font(size))
        tw=(bounds[2]-bounds[0])/SCALE
        px=x if align=='left' else x+(w-tw)/2 if align=='center' else x+w-tw
        py=y+(h-len(lines)*line_h)/2+i*line_h
        draw.text((round(px*SCALE),round(py*SCALE)),line,font=font(size),fill=color)

def line(x1,y1,x2,y2,color='#64748b',width=2,dashed=False,start=False,end=False):
    c = cell('',f'strokeColor={color};strokeWidth={width};dashed={int(dashed)};startArrow={"classic" if start else "none"};endArrow={"classic" if end else "none"};',False)
    g=ET.SubElement(c,'mxGeometry',relative='1',**{'as':'geometry'})
    ET.SubElement(g,'mxPoint',x=str(x1),y=str(y1),**{'as':'sourcePoint'})
    ET.SubElement(g,'mxPoint',x=str(x2),y=str(y2),**{'as':'targetPoint'})
    dx,dy=x2-x1,y2-y1
    length=math.hypot(dx,dy)
    if dashed and length:
        for a in range(0,math.ceil(length),12):
            b=min(a+6,length)
            draw.line(tuple(round(v*SCALE) for v in (x1+dx*a/length,y1+dy*a/length,x1+dx*b/length,y1+dy*b/length)),fill=color,width=width*SCALE)
    else:
        draw.line(tuple(round(v*SCALE) for v in (x1,y1,x2,y2)),fill=color,width=width*SCALE)
    if length:
        ux,uy=dx/length,dy/length
        for yes,px,py,direction in ((start,x1,y1,1),(end,x2,y2,-1)):
            if yes:
                points=[(px,py),(px+direction*ux*10-uy*4,py+direction*uy*10+ux*4),(px+direction*ux*10+uy*4,py+direction*uy*10-ux*4)]
                draw.polygon([(round(a*SCALE),round(b*SCALE)) for a,b in points],fill=color)

def dot(x,y,r=6,color='#2563eb'):
    c=cell('',f'ellipse;fillColor={color};strokeColor={color};',True)
    ET.SubElement(c,'mxGeometry',x=str(x-r),y=str(y-r),width=str(2*r),height=str(2*r),**{'as':'geometry'})
    draw.ellipse(tuple(round(v*SCALE) for v in (x-r,y-r,x+r,y+r)),fill=color)

rect(0,0,W,H,'#f5f7fb','#f5f7fb')
text(42,26,1300,48,'后方长方体：坐标与尺寸核对',32,'#0f172a',bold=True)
text(44,82,1250,36,'原点是 000 轴心  ·  +X 朝抓取物体，−X 朝车后  ·  所有尺寸单位为 mm',21)
rect(35,136,690,510,radius=18)
rect(745,136,620,510,radius=18)
text(58,152,620,38,'① 侧视图：前后 X 与上下 Z',24,'#0f172a',bold=True)
text(772,152,550,38,'② 俯视图：前后 X 与左右 Y',24,'#0f172a',bold=True)

# Side view, true 3 px/mm in both axes.
ox,oz,s=465,337,3
bx1,bx2=ox-70*s,ox-30*s
top,bottom=oz-38.8*s,oz+31.2*s
rect(bx1,top,bx2-bx1,bottom-top,'#ffedd5','#ea580c')
text(bx1+2,top+45,bx2-bx1-4,78,'后方\n障碍物',21,'#9a3412','center')
line(90,bottom,662,bottom,'#94a3b8',3)
line(105,oz,666,oz,'#2563eb',2,end=True)
line(ox,545,ox,202,'#2563eb',2,end=True)
dot(ox,oz)
text(ox+15,oz-33,178,33,'000 轴心 (0,0)',19,'#1d4ed8')
text(90,oz+13,125,31,'车后 / −X',18)
text(622,oz+14,83,62,'+X\n朝物体',17,'#1d4ed8')
text(ox+12,258,130,30,'+Z 向上',18,'#1d4ed8')
line(bx1,top,638,top,'#ea580c',1,True)
text(488,top-31,205,30,'顶面 Z=+38.8',18,'#9a3412')
text(509,bottom+3,205,30,'底板 Z=−31.2',18)
text(bx1-35,bottom+12,70,30,'X=−70',17,'#9a3412','center')
text(bx2-35,bottom+12,70,30,'X=−30',17,'#9a3412','center')
text(ox-20,bottom+12,45,30,'X=0',17,'#1d4ed8','center')
line(218,top,218,bottom,'#ea580c',2,start=True,end=True)
text(114,295,93,42,'H=70',21,'#9a3412','right')
line(bx1,486,bx2,486,'#ea580c',2,start=True,end=True)
text(bx1,488,bx2-bx1,34,'L=40',20,'#9a3412','center')
line(bx2,486,ox,486,'#2563eb',2,start=True,end=True)
text(bx2-6,488,ox-bx2+12,34,'d=30',20,'#1d4ed8','center')
line(550,oz,550,bottom,'#64748b',2,start=True,end=True)
text(556,oz+44,59,39,'31.2',20)
text(118,556,540,66,'最近面：轴心后方 30 → X=−30\n最远面：再向后 40 → X=−70',19)

# Plan view, same scale; symmetry about Y=0 is an assumption.
px,py=1165,390
x1,x2=px-70*s,px-30*s
y1,y2=py-50*s,py+50*s
rect(x1,y1,x2-x1,y2-y1,'#ffedd5','#ea580c')
line(862,py,1323,py,'#2563eb',2,end=True)
line(px,558,px,218,'#64748b',2,end=True)
dot(px,py)
text(px+12,py+7,156,33,'000 (X=0,Y=0)',17,'#1d4ed8')
text(1195,316,140,52,'+X\n朝抓取物体',18,'#1d4ed8')
text(1180,204,135,31,'+Y 左右方向',17)
line(935,y1,935,y2,'#ea580c',2,start=True,end=True)
text(796,359,129,43,'W=100',21,'#9a3412','right')
text(x1+3,y1+55,x2-x1-6,52,'箱体',22,'#9a3412','center')
text(x1+3,y2-68,x2-x1-6,56,'左右\n居中',18,'#9a3412','center')
line(x1,586,x2,586,'#ea580c',2,start=True,end=True)
text(x1,589,x2-x1,31,'L=40',18,'#9a3412','center')
line(x2,586,px,586,'#2563eb',2,start=True,end=True)
text(x2,589,px-x2,31,'d=30',18,'#1d4ed8','center')
text(825,190,490,33,'Y=[−50,+50]；机械臂运动平面为 Y=0',17)

rect(35,668,1330,125,'#eaf2ff','#cbdcf4',16)
text(60,681,1290,45,'箱体范围：X=[−70,−30]   Y=[−50,+50]   Z=[−31.2,+38.8]',25,'#0f172a',bold=True)
text(60,732,1270,40,'此图只核对坐标和尺寸。部件厚度、相机支架外廓及真实碰撞间隙仍需补充。',20)

mxfile=ET.Element('mxfile',host='app.diagrams.net',type='device')
diagram=ET.SubElement(mxfile,'diagram',name='后方障碍物坐标',id='arm-rear-box')
diagram.append(graph)
native=ET.tostring(mxfile,encoding='utf-8',xml_declaration=True)
native_path=OUT/'arm-rear-box.drawio'
native_path.write_bytes(native)
ET.parse(native_path)
png_path=OUT/'arm-rear-box.png'
meta=PngImagePlugin.PngInfo()
meta.add_itxt('Source','arm-rear-box.drawio')
canvas.resize((W,H),Image.Resampling.LANCZOS).save(png_path,pnginfo=meta)
print(native_path)
print(png_path)
