/*
 * Polyhedron_AssimpToCgal.h
 *
 *  Created on: Dec 14, 2016
 *      Author: andrea
 */

#ifndef SRC_POLYHEDRON_ASSIMPTOCGAL_H_
#define SRC_POLYHEDRON_ASSIMPTOCGAL_H_

#include <CGAL/Modifier_base.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <cstdio>

// Can only read triangular meshes

// assimp include files
#include <assimp/Importer.hpp>  // OO version Header!
#include <assimp/postprocess.h>
#include <assimp/scene.h>

using namespace std;

template<class HDS>
class Builder_dae : public CGAL::Modifier_base<HDS> {

private:

  typedef typename HDS::Vertex_handle Vertex_handle;
  typedef typename HDS::Vertex::Point Point;
  typedef typename CGAL::Polyhedron_incremental_builder_3<HDS> Builder;
  string m_filename;

  vector<vector<float> > Vertices_position;
  vector<vector<float> > Vertices_color;
  vector<vector<float> > Vertices_texture_coordinates;
  vector<vector<int> > Facets;
  bool has_texture_coordinates_;

  aiMesh* mesh_;

public:
  bool loadOK;

  Builder_dae(aiMesh* mesh) {
    loadOK = false;
    mesh_ = mesh;
    has_texture_coordinates_ = false;
  }

  ~Builder_dae() {
  }

  void operator()(HDS& hds) {
    Builder builder(hds, true);

    builder.begin_surface(3, 1, 6);
    this->construct(builder);

    if (!this->loadOK)
      return;

    builder.end_surface();

    if (builder.check_unconnected_vertices()) {
      builder.remove_unconnected_vertices();
    }
  }

  bool hasTextureCoordinates() {
    return has_texture_coordinates_;
  }

private:
  /*bool initialize()
   {
   scene = (aiScene*) importer.ReadFile(m_filename, aiProcessPreset_TargetRealtime_Fast);
   // If the import failed, report it
   if (!scene) {
   const char * errorStr = importer.GetErrorString();
   printf("%s\n",errorStr);
   return false;
   }
   return true;
   }*/

  void construct(Builder &builder) {
    for (unsigned int i = 0; i < mesh_->mNumVertices; i++) {
      Vertex_handle vertex = builder.add_vertex(Point(mesh_->mVertices[i].x, mesh_->mVertices[i].y, mesh_->mVertices[i].z));

//      if (mesh_->mColors[0] != NULL)
//        vertex->color(mesh_->mColors[0][i].r, mesh_->mColors[0][i].g, mesh_->mColors[0][i].b);
//
//      if (mesh_->mTextureCoords[0] != NULL) {
//        has_texture_coordinates_ = true;
//        vertex->texture_coordinates(mesh_->mTextureCoords[0][i].x, mesh_->mTextureCoords[0][i].y);
//      }

      has_texture_coordinates_ = false;
    }

    for (unsigned int i = 0; i < mesh_->mNumFaces; i++) {
      builder.begin_facet();
      for (unsigned int j = 0; j < mesh_->mFaces[i].mNumIndices; j++) {
        int index = mesh_->mFaces[i].mIndices[j];
        builder.add_vertex_to_facet(index);
        if (builder.error())
          return;
      }
      builder.end_facet();
    }

    loadOK = true;
  }
};

#endif /* SRC_POLYHEDRON_ASSIMPTOCGAL_H_ */
