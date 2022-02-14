import dtm

s = dtm.dtm("min.xyz","rect.xyz",20,1)
print("let's polygonize now!")
s.polygonize(100,100,50)
s.display();
