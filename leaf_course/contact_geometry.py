"""Evaluation-only support checks on transformed convex lamina solids."""
import numpy as np
from scipy.spatial import ConvexHull

def axes_and_edges(v):
 h=ConvexHull(v);axes=h.equations[:,:3]
 edges=np.array([v[b]-v[a] for f in h.simplices for a,b in zip(f,np.roll(f,-1))])
 edges/=np.linalg.norm(edges,axis=1)[:,None]
 return axes,edges

def separation(a,b):
 na,ea=axes_and_edges(a);nb,eb=axes_and_edges(b)
 cross=np.cross(ea[:,None,:],eb[None,:,:]).reshape(-1,3)
 cross=cross[np.linalg.norm(cross,axis=1)>1e-8]
 axes=np.vstack([na,nb,cross]);axes/=np.linalg.norm(axes,axis=1)[:,None]
 pa=a@axes.T;pb=b@axes.T
 return float(np.maximum(pa.min(0)-pb.max(0),pb.min(0)-pa.max(0)).max())

def support_audit(parts,tolerance=.0003):
 # A leaf can be supported through another leaf. Internal hinge constraints
 # transmit load between patches; this checks external support, not FEM strain.
 grounded={n for n,ps in parts.items() if min(v[:,2].min() for v in ps)<=tolerance}
 graph={n:[] for n in parts};gaps={}
 names=list(parts)
 for i,a in enumerate(names):
  for b in names[i+1:]:
   best=float('inf')
   for va in parts[a]:
    for vb in parts[b]:
     if np.any(va.min(0)-vb.max(0)>tolerance) or np.any(vb.min(0)-va.max(0)>tolerance):continue
     best=min(best,separation(va,vb))
   gaps[a+'|'+b]=None if not np.isfinite(best) else best
   if best<=tolerance:graph[a].append(b);graph[b].append(a)
 supported=set(grounded)
 while True:
  expanded=supported|{b for a in supported for b in graph[a]}
  if expanded==supported:break
  supported=expanded
 return dict(grounded=sorted(grounded),supported=sorted(supported),unsupported=sorted(set(names)-supported),pair_surface_gaps_m=gaps)
