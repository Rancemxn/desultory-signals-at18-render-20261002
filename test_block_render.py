"""Exercise real SkSL rendering, seeking and non-block chart compatibility."""
import numpy as np
import skia
from block_area import BlockAreas
from block_render import BlockRenderer


def check():
    block=dict(bottomLeftPercentage={'x':.2,'y':.2},topRightPercentage={'x':.8,'y':.8},
               appearTime=0,enableTime=1,disableTime=3,disappearTime=4,isSubtract=False,
               moveEvents=[dict(time=1,endPosition={'x':.5,'y':.5},easeTypeX=0,easeTypeY=0),
                           dict(time=3,endPosition={'x':.7,'y':.5},easeTypeX=0,easeTypeY=0)])
    renderer=BlockRenderer(BlockAreas([block]),320,180,use_gpu=False)
    def frame(t):
        surface=skia.Surface(320,180);surface.getCanvas().clear(skia.Color(30,40,50))
        renderer.draw(surface.getCanvas(),t)
        return surface.makeImageSnapshot().toarray().copy()
    assert np.array_equal(frame(-1),frame(5))
    assert not np.array_equal(frame(.2),frame(1.2))
    first=frame(1.2)
    assert not np.array_equal(first,frame(2.2))
    assert np.array_equal(first,frame(1.2)), 'Seeking must not accumulate block effects'
    assert np.all(first[:,:,3]==255)
    print('Block render shaders, movement, phases and seeking checks passed')


if __name__=='__main__':check()
