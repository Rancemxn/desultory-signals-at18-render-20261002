"""Raster counterpart of Phigros 4.0.1's low-resolution block effects.

Geometry shares block_area.py with the planner. The SkSL effects reproduce the
idle active/ready/disabled material paths; no blocked-touch feedback is synthesized
for the validated autoplay plan. GPU/color-space differences are documented.
"""
from pathlib import Path
import math
import numpy as np
import skia
from block_area import compose


ASSETS=Path(__file__).with_name('block_assets')


def _path(geometry):
    path=skia.Path()
    path.setFillType(skia.PathFillType.kEvenOdd)
    polygons = [geometry] if geometry.geom_type=='Polygon' else getattr(geometry,'geoms',())
    for polygon in polygons:
        if polygon.geom_type!='Polygon':
            continue
        for ring in (polygon.exterior,*polygon.interiors):
            xy=list(ring.coords)
            if not xy: continue
            path.moveTo(*xy[0])
            for p in xy[1:]: path.lineTo(*p)
            path.close()
    return path


def _dilate(a):
    padded=np.pad(a,1,mode='edge')
    h,w=a.shape
    return np.maximum.reduce([padded[y:y+h,x:x+w] for y in range(3) for x in range(3)])


class BlockRenderer:
    def __init__(self,blocks,width,height):
        self.blocks,self.width,self.height=blocks,width,height
        self.size=(max(1,width//8),max(1,height//8))
        self.effect_size=tuple(n*2 for n in self.size)
        self.scene_size=(max(1,width//6),max(1,height//6))
        self.effects={name:skia.RuntimeEffect.MakeForShader((ASSETS/f'{name}.sksl').read_text())
                      for name in ('compose','active','disabled')}
        self.noise=skia.Image.open(str(ASSETS/'displace.png')).makeShader(
            skia.TileMode.kRepeat,skia.TileMode.kRepeat,skia.SamplingOptions(skia.FilterMode.kLinear))
        self.spark=skia.Image.open(str(ASSETS/'spark.png')).makeShader(
            skia.TileMode.kRepeat,skia.TileMode.kRepeat,skia.SamplingOptions(skia.FilterMode.kLinear))
        self.black=skia.Shaders.Color(skia.ColorBLACK)

    def mask(self,geometry):
        s=skia.Surface(*self.size)
        c=s.getCanvas();c.clear(skia.ColorBLACK)
        c.scale(self.size[0]/self.blocks.width,self.size[1]/self.blocks.height)
        c.drawPath(_path(geometry),skia.Paint(Color=skia.ColorWHITE,AntiAlias=False))
        return s.makeImageSnapshot()

    @staticmethod
    def sampler(image,linear=False):
        return image.makeShader(skia.TileMode.kClamp,skia.TileMode.kClamp,
            skia.SamplingOptions(skia.FilterMode.kLinear if linear else skia.FilterMode.kNearest))

    def shader(self,name,seconds,children,**uniforms):
        b=skia.RuntimeShaderBuilder(self.effects[name])
        b.setUniform('resolution',(float(self.width),float(self.height)))
        b.setUniform('time',float(seconds))
        for key,value in uniforms.items():
            b.setUniform(key,tuple(float(n) for n in value) if isinstance(value,tuple) else value)
        for key,value in children.items(): b.setChild(key,value)
        return b.makeShader()

    def draw(self,canvas,seconds):
        phases={'active':[],'ready':[],'disabled':[]}
        for b in self.blocks.areas:
            phase=b.phase(seconds)
            if phase in phases: phases[phase].append(b.rectangle(seconds))
        if not any(phases.values()): return
        geometries={k:compose(v) for k,v in phases.items()}
        # The APK uses 1/8-size point-filtered masks, 1/4-size effects and a
        # 1/6-size scene-color capture. Keep that characteristic blocky edge.
        raw=self.mask(geometries['active'])
        mask_surface=skia.Surface(*self.size)
        compose_shader=self.shader('compose',seconds,{'mask':self.sampler(raw),'noise':self.noise},resolution=self.size)
        mask_surface.getCanvas().drawPaint(skia.Paint(Shader=compose_shader))
        mask=mask_surface.makeImageSnapshot()
        pixels=mask.toarray(colorType=skia.ColorType.kRGBA_8888_ColorType)[:,:,0].astype(np.float32)/255
        pixels=np.repeat(np.repeat(pixels,2,axis=0),2,axis=1)
        edge=np.clip(_dilate(pixels)-pixels,0,1)
        glow=np.zeros_like(pixels);grown=pixels
        denom=sum(n**2.65 for n in range(1,7))
        for k in range(6):
            weight=(6-k)**2.65/denom
            if weight<.01: break
            expanded=_dilate(grown)
            glow+=np.clip(expanded-grown,0,1)*(1-pixels)*weight
            grown=expanded
        rgba=np.zeros((*pixels.shape,4),dtype=np.uint8)
        rgba[:,:,0]=np.rint(edge*255).astype(np.uint8)
        rgba[:,:,1]=np.rint(np.clip(glow,0,1)*255).astype(np.uint8)
        rgba[:,:,3]=255
        effect=skia.Image.fromarray(rgba,colorType=skia.ColorType.kRGBA_8888_ColorType)
        disabled=self.mask(geometries['disabled'])
        ready=self.mask(geometries['ready'])
        surface=canvas.getSurface()
        if surface is None: raise ValueError('Block effects require a raster surface')
        scene_surface=skia.Surface(*self.scene_size)
        scene_surface.getCanvas().drawImageRect(surface.makeImageSnapshot(),skia.Rect.MakeWH(*self.scene_size),
            skia.SamplingOptions(skia.FilterMode.kLinear))
        children={'mask':self.sampler(mask),'effects':self.sampler(effect,True),
                  'ready':self.sampler(ready),'scene':self.sampler(scene_surface.makeImageSnapshot()),
                  'noise':self.noise,'spark':self.spark}
        active=self.shader('active',seconds,children,maskSize=self.size,effectSize=self.effect_size,sceneSize=self.scene_size)
        canvas.drawPaint(skia.Paint(Shader=active))
        if not geometries['disabled'].is_empty:
            shader=self.shader('disabled',seconds,{'mask':self.sampler(disabled),'noise':self.noise,'spark':self.spark},maskSize=self.size)
            canvas.drawPaint(skia.Paint(Shader=shader,BlendMode=skia.BlendMode.kPlus))
