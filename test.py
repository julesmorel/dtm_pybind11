import dtm

s = dtm.dtm("min.xyz","rect.xyz",20,1.)
print("let's deform now!")
s.applyDeformableModel(1,0.5)
s.polygonize(200,200,50)
s.display();
