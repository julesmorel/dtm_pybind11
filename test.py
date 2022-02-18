import dtm

s = dtm.dtm("data/TLS_ground.xyz","data/rect.xyz",20,1.)
s.applyDeformableModel(1,0.5)
s.polygonize(200,200,50)
s.exportDTM("dtm.obj")
s.display();
