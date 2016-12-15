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
class MeshBuilder : public CGAL::Modifier_base<HDS> {

private:

  typedef typename HDS::Vertex_handle Vertex_handle;
  typedef typename HDS::Vertex::Point Point;
  typedef typename CGAL::Polyhedron_incremental_builder_3<HDS> Builder;
  aiMesh* mesh_;

public:
  bool loadOK;

  MeshBuilder(aiMesh* mesh) {
    loadOK = false;
    mesh_ = mesh;
  }

  ~MeshBuilder() {
  }

  void operator()(HDS& hds) {
    Builder builder(hds, true);

//    builder.begin_surface(mesh_->mNumVertices, mesh_->mNumFaces);
    builder.begin_surface(3, 1, 6);
    this->construct(builder);

    if (!this->loadOK)
      return;

    builder.end_surface();

    if (builder.check_unconnected_vertices()) {
      builder.remove_unconnected_vertices();
    }
  }

private:

  void construct(Builder &builder) {

    for (unsigned int i = 0; i < mesh_->mNumVertices; i++) {
      Vertex_handle vertex = builder.add_vertex(Point((float)mesh_->mVertices[i].x, (float)mesh_->mVertices[i].y, (float)mesh_->mVertices[i].z));
    }

    for (unsigned int i = 0; i < mesh_->mNumFaces; i++) {
      builder.begin_facet();
//      builder.add_vertex_to_facet(mesh_->mFaces[i].mIndices[0]);
//      builder.add_vertex_to_facet(mesh_->mFaces[i].mIndices[1]);
//      builder.add_vertex_to_facet(mesh_->mFaces[i].mIndices[2]);
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
