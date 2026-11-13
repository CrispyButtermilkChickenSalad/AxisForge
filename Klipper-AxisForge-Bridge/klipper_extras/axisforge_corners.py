"""Corner speed policies and bounded geometry, with no Klipper dependencies.

Klipper SCV and Marlin JD are junction-speed policies, not firmware emulation.
The robot's interpolation, joint limits and tracking checks remain authoritative.
"""
import math

DEFAULTS = {'policy':'exact', 'square_corner_velocity':1., 'junction_deviation_mm':.01,
            'blend_tolerance_mm':.05, 'blend_speed_mm_s':5.}
POLICIES = ('exact', 'klipper', 'marlin', 'blend')


def options(value=None):
    result = dict(DEFAULTS)
    if value is not None:
        if not isinstance(value,dict) or set(value)-set(result):
            raise ValueError('Unknown motion options')
        result.update(value)
    if result['policy'] not in POLICIES:
        raise ValueError('Unknown corner policy')
    for key,maximum in (('square_corner_velocity',20.),('junction_deviation_mm',1.),
                        ('blend_tolerance_mm',.2),('blend_speed_mm_s',20.)):
        v=result[key]
        if type(v) not in (int,float) or not math.isfinite(v) or not 0<=v<=maximum:
            raise ValueError('%s must be a finite number from 0 to %g' % (key,maximum))
        result[key]=float(v)
    return result


def junction_limit(cosine, left_length, right_length, left_accel, right_accel,
                   settings, nominal_accel):
    """SCV/JD centripetal approximation; lengths are XYZ millimetres.

    SCV is converted to JD at the configured acceleration, as in Klipper.
    The half-segment contact bound avoids excessive speed on tiny segments.
    Marlin uses its JD radius and small-segment arc bound (for <1 mm moves).
    """
    cosine=max(-1.,min(1.,cosine))
    if cosine > 1.-1e-12: return math.inf
    if cosine < -1.+1e-12: return 0.
    sin_half=math.sqrt((1.+cosine)/2.)
    cos_half=math.sqrt((1.-cosine)/2.)
    jd=(settings['square_corner_velocity']**2*(math.sqrt(2.)-1.)/nominal_accel
        if settings['policy']=='klipper' else settings['junction_deviation_mm'])
    limit=jd*min(left_accel,right_accel)*sin_half/(1.-sin_half)
    if settings['policy']=='klipper':
        limit=min(limit,.5*left_length*left_accel*sin_half/cos_half,
                  .5*right_length*right_accel*sin_half/cos_half)
    else:
        # JD_HANDLE_SMALL_SEGMENTS: turn angle <45 degrees, next move <1 mm.
        if right_length < 1. and cosine > .7071067811865476:
            limit=min(limit,right_length*min(left_accel,right_accel)/math.acos(cosine))
    return math.sqrt(max(0.,limit))


def sub(a,b): return [x-y for x,y in zip(a,b)]
def add(a,b): return [x+y for x,y in zip(a,b)]
def scale(a,k): return [x*k for x in a]
def norm(a): return math.sqrt(sum(x*x for x in a))
def dot(a,b): return sum(x*y for x,y in zip(a,b))
def lerp(a,b,t): return [x+(y-x)*t for x,y in zip(a,b)]


def split_bezier(points):
    left,right=[points[0]],[points[-1]]
    while len(points)>1:
        points=[lerp(a,b,.5) for a,b in zip(points,points[1:])]
        left.append(points[0]); right.append(points[-1])
    return left,list(reversed(right))


def curve_leaves(points,depth=5):
    if not depth: return [points]
    left,right=split_bezier(points)
    return curve_leaves(left,depth-1)+curve_leaves(right,depth-1)


def curvature_bound(points):
    """Bound curvature using derivative control hulls on a short Bezier span."""
    derivative=[scale(sub(b,a),5.) for a,b in zip(points,points[1:])]
    second=[scale(sub(b,a),4.) for a,b in zip(derivative,derivative[1:])]
    chord=sub(points[-1],points[0]); direction=scale(chord,1./norm(chord))
    lower=min(dot(d,direction) for d in derivative)
    return max(norm(d) for d in second)/lower**2 if lower>1e-12 else math.inf


def round_run(moves,settings,limits,max_records=20000):
    """Quintic corner curves flattened to a bounded, shared XYZ/E polyline.

    Half of the tolerance is reserved for geometric displacement; robot
    preflight checks actual joint interpolation against the original path at
    the full tolerance. Source filament totals are preserved on each half-curve
    and distributed by its chord length, so the native PA queue sees that flow.
    Changing ABC, reversals, E sign and PA eligibility changes retain a stop.
    """
    tolerance=settings['blend_tolerance_mm']
    if not tolerance or not settings['blend_speed_mm_s']: return moves
    lengths=[math.dist(m['start'][:3],m['end'][:3]) for m in moves]
    trims=[0.]*(len(moves)+1); curves={}
    for i in range(1,len(moves)):
        left,right=moves[i-1:i+1]
        if min(lengths[i-1:i+1])<1e-8: continue
        if left['start'][3:]!=left['end'][3:] or right['start'][3:]!=right['end'][3:]: continue
        sign=lambda e:(e>0)-(e<0)
        if sign(left['delta_e'])!=sign(right['delta_e']): continue
        if left['delta_e']<0: continue
        # Preserve native PA's XY-extrusion eligibility across a rounded corner.
        if (left['start'][:2]!=left['end'][:2]) != (right['start'][:2]!=right['end'][:2]): continue
        u=scale(sub(left['end'][:3],left['start'][:3]),1./lengths[i-1])
        v=scale(sub(right['end'][:3],right['start'][:3]),1./lengths[i])
        cosine=dot(u,v)
        if cosine>1.-1e-10 or cosine<-.95: continue
        # Zero endpoint curvature; collinear control triples meet each line tangentially.
        controls=[scale(u,-1.),scale(u,-.5),[0.,0.,0.],[0.,0.,0.],scale(v,.5),v]
        leaves=curve_leaves(controls)
        points=[leaves[0][0]]+[leaf[-1] for leaf in leaves]
        distances=[norm(sub(b,a)) for a,b in zip(points,points[1:])]
        half=len(distances)//2
        references=[]
        for lo,hi,start,end in ((0,half,scale(u,-1.),[0.,0.,0.]),
                                (half,len(distances),[0.,0.,0.],v)):
            total=sum(distances[lo:hi]); cumulative=0.
            for j in range(lo,hi):
                references.append(lerp(start,end,cumulative/total))
                cumulative+=distances[j]
        references.append(v)
        deviation=max(norm(sub(p,r)) for p,r in zip(points,references))
        trim=min(lengths[i-1]/4.,lengths[i]/4.,.5*tolerance/deviation)
        if trim<1e-7: continue
        curvature=max(curvature_bound(leaf) for leaf in leaves)/trim
        speed=min(settings['blend_speed_mm_s'],math.sqrt(.5*limits['linear_acceleration']/curvature))
        trims[i]=trim
        if len(moves)+(len(curves)+1)*len(distances)>max_records:
            raise ValueError('Rounded run exceeds operation limit; add M400')
        curves[i]=(points,references,distances,trim,speed)

    output=[]
    for i,move in enumerate(moves):
        length=lengths[i]
        f0=trims[i]/length if length else 0.
        f1=1.-trims[i+1]/length if length else 1.
        line=dict(move,start=lerp(move['start'],move['end'],f0),
                  end=lerp(move['start'],move['end'],f1),
                  e_start=move['e_start']+move['delta_e']*f0,
                  delta_e=move['delta_e']*(f1-f0))
        output.append(line)
        if i+1 not in curves: continue
        points,refs,distances,trim,speed=curves[i+1]
        corner=move['end']; right=moves[i+1]; half=len(distances)//2
        for lo,hi,source,amount,origin in (
            (0,half,move,move['delta_e']*trim/length,
             move['e_start']+move['delta_e']*(1.-trim/length)),
            (half,len(distances),right,right['delta_e']*trim/lengths[i+1],right['e_start'])):
            total=sum(distances[lo:hi]); used=0.
            for j in range(lo,hi):
                e=amount*distances[j]/total if j+1<hi else amount-used
                pose=lambda p:add(corner[:3],scale(p,trim))+corner[3:]
                record=dict(source,start=pose(points[j]),end=pose(points[j+1]),
                            e_start=origin+used,delta_e=e,
                            reference_start=pose(refs[j]),reference_end=pose(refs[j+1]),
                            blend_segment=True,_curve_speed=speed,
                            _feed=min(move['_feed'],right['_feed']))
                output.append(record); used+=e
    return output
