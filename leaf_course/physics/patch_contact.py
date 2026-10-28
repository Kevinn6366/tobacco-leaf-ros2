"""Patch only certified convex lamina collisions in pinned Gazebo 7.8.0."""
from pathlib import Path
import subprocess,sys
root=Path(sys.argv[1]);rel='bullet-featherstone/src/SDFFeatures.cc'
s=subprocess.check_output(['git','-C',str(root),'show','189471c86fa06ffad9c9bdb199f6928eb2fc02f3:'+rel],text=True)
anchor='''        this->triangleMeshes.push_back(std::make_unique<btTriangleMesh>());'''
assert s.count(anchor)==1
patch='''        // leaf_course: these named solids are verified closed and convex
        // before launch. Use the exact vertices, no voxel decomposition and
        // no GImpact triangle seams. Preserve all other model collisions.
        if (_collision.Name() == "lamina_contact" || _collision.Name().find("pinch_bevel") != std::string::npos)
        {
          this->meshesConvex.push_back(std::make_unique<btConvexHullShape>(
              &(convertedVerts[0].getX()), convertedVerts.size()));
          auto *lamina = this->meshesConvex.back().get();
          lamina->setMargin(btScalar(0.00005));
          lamina->recalcLocalAabb();
          compoundShape->addChildShape(btTransform::getIdentity(), lamina);
          continue;
        }

'''
(root/rel).write_text(s.replace(anchor,patch+anchor))
# Continuous sheet neighbours share an edge or vertex. Their contact is handled
# by the hinge topology; treating those seams as friction contacts locks bending.
# Only exclude neighbours within the SAME multibody, never different leaves.
s=(root/rel).read_text()
a='''  linkInfo->shape = std::make_unique<btCompoundShape>();'''
assert s.count(a)==1
s=s.replace(a,'''  if (modelInfo->name.rfind("leaf_", 0) == 0)
  {
    const auto mark = linkInfo->name.rfind("_p");
    if (mark != std::string::npos)
    {
      int patch = std::stoi(linkInfo->name.substr(mark + 2));
      if (patch >= 0 && patch < 9) linkInfo->collider->laminaPatch = patch;
    }
  }

'''+a)
(root/rel).write_text(s)
for filename in ['Base.hh','Base.cc']:
 rel2='bullet-featherstone/src/'+filename
 s=subprocess.check_output(['git','-C',str(root),'show','189471c86fa06ffad9c9bdb199f6928eb2fc02f3:'+rel2],text=True)
 if filename=='Base.hh':
  s=s.replace('  public: uint16_t collideBitmask', '  public: int laminaPatch = -1;\n\n  public: uint16_t collideBitmask')
 else:
  anchor='''  if (col0 && col1)
  {'''
  assert s.count(anchor)==2
  s=s.replace(anchor,anchor+'''
    if (col0->m_multiBody == col1->m_multiBody &&
        col0->laminaPatch >= 0 && col1->laminaPatch >= 0)
    {
      const int rows[3] = {0, -1, 1};
      const int a = col0->laminaPatch, b = col1->laminaPatch;
      if (std::abs(a / 3 - b / 3) <= 1 &&
          std::abs(rows[a % 3] - rows[b % 3]) <= 1)
        return false;
    }
''')
 (root/rel2).write_text(s)
# Certified convex laminae use normal rigid contact constraints, not GImpact's
# mesh-softening branch (kp=1e15, kd=1e14). Restore Bullet's ordinary ERP and
# increase iterations for thin-sheet contact; this engine is project-local.
p=root/'bullet-featherstone/src/SDFFeatures.cc';s=p.read_text();s=s.replace('if (geom->MeshShape())\n      {','if (geom->MeshShape() && _collision.Name() != "lamina_contact" && _collision.Name().find("pinch_bevel") == std::string::npos)\n      {');p.write_text(s)
p=root/'bullet-featherstone/src/Base.cc';s=p.read_text().replace('m_erp2 = btScalar(0.02)','m_erp2 = btScalar(0.2)').replace('m_numIterations = 50u','m_numIterations = 100u');p.write_text(s)
# Generate clipped face manifolds instead of relying on one GJK witness point.
# This is essential for torque stability of a thin lamina between broad pads.
p=root/'bullet-featherstone/src/SDFFeatures.cc';s=p.read_text().replace('lamina->recalcLocalAabb();','lamina->recalcLocalAabb();\n          lamina->initializePolyhedralFeatures();');s=s.replace('shape = std::make_unique<btBoxShape>(halfExtents);','shape = std::make_unique<btBoxShape>(halfExtents);\n    static_cast<btBoxShape *>(shape.get())->initializePolyhedralFeatures();');p.write_text(s)
p=root/'bullet-featherstone/src/Base.cc';s=p.read_text().replace('this->world->getSolverInfo().m_numIterations = 100u;','this->world->getSolverInfo().m_numIterations = 100u;\n  this->world->getDispatchInfo().m_enableSatConvex = true;');p.write_text(s)
