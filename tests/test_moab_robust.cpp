
#include "test_files.h"

#include "assert.h"

#include "moab/Core.hpp"
#include "moab/CartVect.hpp"
#include "moab/Range.hpp"

#include "MBVH.h"

#include "testutil.hpp"

int main(int argc, char** argv) {
  
  moab::ErrorCode rval;

  //create a new MOAB instance
  moab::Interface* mbi = new moab::Core();

  std::string filename;
  
  // check for a user-specified file
  if (argc > 1) {
    filename = std::string(argv[1]);
  }
  else {
    filename = TEST_SPHERE;
  }
    
  std::cout << "Loading file: " << filename << std::endl;

  rval = mbi->load_file(filename.c_str());
  MB_CHK_SET_ERR(rval, "Failed to load file: " << filename);

  

  return rval;
}

