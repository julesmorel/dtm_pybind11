#include "dtm.h"

int main(int argc, char *argv[]){
  std::string filenameMin,filenameRect;
  int numberMin;
  float sizeMin;
  if (argc == 5) {
    filenameMin = argv[1];
    filenameRect = argv[2];
    numberMin = std::stoi(argv[3]);
    sizeMin = std::stod(argv[4]);
  }else{
    std::cout<<"Please specify a file to process"<<std::endl;
    exit(0);
  }
  dtm d(filenameMin,filenameRect,numberMin,sizeMin);
  d.polygonize(100,100,50);
  d.display();
}
