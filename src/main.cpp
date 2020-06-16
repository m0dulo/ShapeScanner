#include "scanner.h"
#include "Argument_helper.h"
#include <memory>

int main(int argc, char* argv[]) {
  std::string img = "";
  dsr::Argument_helper ah;
  ah.new_named_string("data", "data_name", "named_string", "named_string",
                      img);
  ah.process(argc, argv);

  time_t timeStamp;
  int threshold = 50;
  unique_ptr<ShapeScanner> scanner(new ShapeScanner(img, threshold));
  scanner->scanShapes();
  Mat res = scanner->return_dst();
//  Mat ed = scanner->return_ed();
//  imshow("ed", ed);
  imwrite("res.jpg", res);
//  waitKey(0);
  return 0;
}