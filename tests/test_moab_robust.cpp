
#include "test_files.h"

#include "assert.h"

#include "MBTagConventions.hpp"
#include "moab/Core.hpp"
#include "moab/CartVect.hpp"
#include "moab/Range.hpp"

#include "MBVH.h"

#include "testutil.hpp"

moab::ErrorCode get_all_volumes(moab::Interface* mbi, moab::Range& volumes);
moab::ErrorCode get_all_surfaces(moab::Interface *mbi, moab::Range& surfaces);

moab::ErrorCode get_triangles_on_volume(moab::Interface* mbi, moab::EntityHandle volume, std::vector<moab::EntityHandle>& triangles);

moab::ErrorCode get_triangles_on_surface(moab::Interface* mbi, moab::EntityHandle surface, std::vector<moab::EntityHandle> &triangles);

std::vector<TriangleRef> create_build_triangles(moab::Interface* mbi, std::vector<moab::EntityHandle> triangles);

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

  std::cout << "Loading complete" << std::endl;

  // retrieve the volumes and surfaces
  moab::Range volumes;
  moab::Range surfaces;
  rval = get_all_volumes(mbi, volumes);
  MB_CHK_SET_ERR(rval, "Failed to retrieve volume meshsets");

  rval = get_all_surfaces(mbi, surfaces);
  MB_CHK_SET_ERR(rval, "Failed to retrieve surface meshsets");

  // working with only one volume (for now)
  assert(volumes.size() == 1);
  
  // get all of the triangles in the volume
  std::vector<moab::EntityHandle> volume_triangles;
  rval = get_triangles_on_volume(mbi, volumes[0], volume_triangles);
  MB_CHK_SET_ERR(rval, "Failed to retrieve triangles for the volume with handle: " << volumes );
  
  // create build triangles
  std::vector<TriangleRef> tri_refs = create_build_triangles(mbi, volume_triangles);

  TriangleBVH* TBVH = new TriangleBVH();

  BuildSettings settings;

  BuildStateTri bs = BuildStateTri(0, tri_refs);

  NodeRef* root = TBVH->Build(settings, bs);

  return rval;
}


std::vector<TriangleRef> create_build_triangles(moab::Interface* mbi, std::vector<moab::EntityHandle> triangles) {

  std::vector<TriangleRef> tris;
  for(unsigned int i = 0; i < triangles.size(); i++) {

    TriangleRef *t = new TriangleRef(triangles[i], mbi);

    tris.push_back(*t);
  }
  
  return tris;
}

moab::ErrorCode get_all_volumes(moab::Interface* mbi, moab::Range& volumes){
  // get the entities tagged with dimension & type
  moab::ErrorCode rval;
  
  int three[1] = {3};
  const void* const dim[1] = {three};
  moab::Tag geom_tag;

  // get the tag handle
  rval = mbi->tag_get_handle(GEOM_DIMENSION_TAG_NAME, 1, moab::MB_TYPE_INTEGER, geom_tag,
			     moab::MB_TAG_SPARSE|moab::MB_TAG_CREAT);
  // get the entities tagged with dimension & type 
  rval = mbi->get_entities_by_type_and_tag(0,moab::MBENTITYSET,&geom_tag,dim,1,volumes);

  if (rval != moab::MB_SUCCESS )
    {
      std::cout << "Failed to get volumes from file " << std::endl;
    }
  else
    {
      std::cout << "Found " << volumes.size() << " volumes" << std::endl;
    }

  return rval;
}


moab::ErrorCode get_all_surfaces(moab::Interface *mbi, moab::Range& surfaces){
  // get the entities tagged with dimension & type
  moab::ErrorCode rval;

  int two[1] = {2};
  const void* const dim[1] = {two};
  moab::Tag geom_tag;

  // get the tag handle
  rval = mbi->tag_get_handle(GEOM_DIMENSION_TAG_NAME, 1, moab::MB_TYPE_INTEGER, geom_tag,
			     moab::MB_TAG_SPARSE|moab::MB_TAG_CREAT);
  // get the entities tagged with dimension & type 
  rval = mbi->get_entities_by_type_and_tag(0,moab::MBENTITYSET,&geom_tag,dim,1,surfaces);

  if (rval != moab::MB_SUCCESS )
    {
      std::cout << "Failed to get surfaces from file " << std::endl;
    }
  else
    {
      std::cout << "Found " << surfaces.size() << " surfaces" << std::endl;
    }

  return rval;
}


moab::ErrorCode get_triangles_on_surface(moab::Interface* mbi, moab::EntityHandle surface, std::vector<moab::EntityHandle> &triangles)
{
  // get the entities tagged with dimension & type
  moab::ErrorCode rval;

  // get the volume id tag
  moab::Tag id_tag;
  int id; // id number of the volume

  // get the id tag handle
  rval = mbi->tag_get_handle(GLOBAL_ID_TAG_NAME, 1, moab::MB_TYPE_INTEGER, id_tag,
			     moab::MB_TAG_SPARSE|moab::MB_TAG_CREAT);
  rval = mbi->tag_get_data(id_tag,&(surface),1,&id);
  

  rval = mbi->get_entities_by_type(surface, moab::MBTRI,triangles);
  
  std::cout << "Surface " << id << " has " << triangles.size() << " triangles"  << std::endl;

  return rval;
}

/* get the triangles for the given volume */
moab::ErrorCode get_triangles_on_volume(moab::Interface* mbi, moab::EntityHandle volume, std::vector<moab::EntityHandle>& triangles)
{
  // get the entities tagged with dimension & type
  moab::ErrorCode rval;

  // get the volume id tag
  moab::Tag id_tag;
  int id; // id number of the volume

  // get the id tag handle
  rval = mbi->tag_get_handle(GLOBAL_ID_TAG_NAME, 1, moab::MB_TYPE_INTEGER, id_tag,
			     moab::MB_TAG_SPARSE|moab::MB_TAG_CREAT);
  rval = mbi->tag_get_data(id_tag,&(volume),1,&id);
  

  moab::Range child_surface_sets;
  // get the child sets are all the surfaces
  rval = mbi->get_child_meshsets(volume,child_surface_sets);

  moab::Range::iterator surf_it;
  // moab ranges are additive, so it gets appended to every time
  for ( surf_it = child_surface_sets.begin() ; surf_it != child_surface_sets.end() ; ++surf_it)
    {
      rval = mbi->get_entities_by_type(*surf_it,moab::MBTRI,triangles);
    }
  std::cout << "Volume " << id << " has " << triangles.size() << " triangles"  << std::endl;

  return rval;
}
