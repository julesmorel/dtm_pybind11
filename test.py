import dtm

s = dtm.dtm("data/filtered_Gedi_018_ground.xyz","data/filtered_Gedi_018_ground_bbox_40x40.xyz",10,0.2)
s.polygonize(1000,1000,100)
s.exportDTM("dtm2.obj")
s.applyDeformableModel(5,0.9)
s.polygonize(1000,1000,100)
s.exportDTM("dtm_def2.obj")
s.display();
