#include "kdtree.h"

BoundingBox::BoundingBox() { }

BoundingBox::BoundingBox(Vector<FLOAT,3> min, Vector<FLOAT,3> max) 
 : min(min), max(max) { }

void BoundingBox::split(BoundingBox & left, BoundingBox & right) {
  // from here
  FLOAT lengthX = std::abs(max[0]-min[0]);
  FLOAT lengthY = std::abs(max[1]-min[1]);
  FLOAT lengthZ = std::abs(max[2]-min[2]);
  //CHECK X AXIS
  if(lengthX >=lengthY && lengthX > lengthZ) {
	left.min = Vector<FLOAT,3> {min[0], min[1], min[2]};
	left.max = Vector<FLOAT,3> {min[0] + (lengthX/2), max[1], max[2]};
    right.min = Vector<FLOAT,3> {min[0] + (lengthX/2), min[1], min[2]};
    right.max = Vector<FLOAT,3> {max[0], max[1], max[2]};
  //CHECK Y AXIS
  } else if (lengthY > lengthZ) {
    left.min = Vector<FLOAT,3> {min[0], min[1], min[2]};
    left.max = Vector<FLOAT,3> {max[0], min[1] + (lengthY/2), max[2]};
    right.min = Vector<FLOAT,3> {min[0], min[1] + (lengthY/2), min[2]};
    right.max = Vector<FLOAT,3> {max[0], max[1], max[2]};
  //OTHERWISE SPLIT ON Z AXIS
  } else {
    left.min = Vector<FLOAT,3> {min[0], min[1], min[2]};
    left.max = Vector<FLOAT,3> {max[0], max[1], min[2] + (lengthZ/2)};
    right.min = Vector<FLOAT,3> {min[0], min[1], min[2] + (lengthZ/2)};
    right.max = Vector<FLOAT,3> {max[0], max[1], max[2]};
  }
  // to here
}

bool BoundingBox::contains(Vector<FLOAT, 3> v) {
  // from here
  return v[0] >= min[0] && v[0] <= max[0] && 
         v[1] >= min[1] && v[1] <= max[1] && 
		 v[2] >= min[2] && v[2] <= max[2];
  // to here
}

bool BoundingBox::contains(Triangle<FLOAT> *triangle) {
  // from here
  return contains(triangle->p1) || contains(triangle->p2) || contains(triangle->p3);
  // to here
}

bool BoundingBox::intersects(Vector<FLOAT,3> eye, Vector<FLOAT, 3> direction) {
    // slab test implementation
    FLOAT tmin[3] = { (min[0] - eye[0]) / direction[0],
                      (min[1] - eye[1]) / direction[1],
                      (min[2] - eye[2]) / direction[2] };
    FLOAT tmax[3] = { (max[0] - eye[0]) / direction[0],
                      (max[1] - eye[1]) / direction[1],
                      (max[2] - eye[2]) / direction[2] };
    FLOAT tminimum = std::min(tmin[0], tmax[0]);
    FLOAT tmaximum = std::max(tmin[0], tmax[0]);
    tminimum = std::max(tminimum, std::min(tmin[1], tmax[1]) );
    tmaximum = std::min(tmaximum, std::max(tmin[1], tmax[1]) );
    tminimum = std::max(tminimum, std::min(tmin[2], tmax[2]) );
    tmaximum = std::min(tmaximum, std::max(tmin[2], tmax[2]) );

    return tmaximum >= tminimum;
}


KDTree::~KDTree() {
  delete left;
  delete right;
}

KDTree * KDTree::buildTree(KDTree * tree, std::vector< Triangle<FLOAT> *> & triangles) {
  // from here
  if (triangles.size() < MAX_TRIANGLES_PER_LEAF) {
    for (auto const &triangle : triangles) {
      this->triangles.push_back(triangle);
    }
    return nullptr;
  }

  tree->left = new KDTree();
  BoundingBox leftBox = BoundingBox({0,0,0},{0,0,0});
  tree->right = new KDTree();
  BoundingBox rightBox = BoundingBox({0,0,0},{0,0,0});

  tree->box.split(rightBox, leftBox);
  tree->left->box = leftBox;
  tree->right->box = rightBox;

  std::vector<Triangle<FLOAT> *> leftTriangles;
  std::vector<Triangle<FLOAT> *> rightTriangles;

  for (auto const &triangle : triangles) {
    bool inLeftBox = tree->left->box.contains(triangle);
    bool inRightBox = tree->right->box.contains(triangle);
    if (inLeftBox && !inRightBox) {
      leftTriangles.push_back(triangle);
    } else if (!inLeftBox && inRightBox) {
      rightTriangles.push_back(triangle);
    } else if (inLeftBox && inRightBox) {
      this->triangles.push_back(triangle);
    } else {
      std::cerr << "ERROR: Triangle neither in left nor in right bounding box: "
        << triangle->p1 << ", "
        << triangle->p2 << ", "
        << triangle->p3
        << std::endl;
    }
  }

  tree->left->buildTree(tree->left, leftTriangles);
  tree->right->buildTree(tree->right, rightTriangles);
  // to here
  return tree;
}

KDTree *  KDTree::buildTree(std::vector< Triangle<FLOAT> *> & triangles)  {
  KDTree * root = new KDTree();
  // from here
  Vector<FLOAT, 3> boxMin = {256, 256, 256};
  Vector<FLOAT, 3> boxMax = {-256, -256, -256};
  for (auto const &triangle : triangles) {
    for (unsigned int i = 0; i < 3; i++) {
      boxMin[i] = std::min(std::min(std::min(boxMin[i], triangle->p1[i]), triangle->p2[i]), triangle->p3[i]);
      boxMax[i] = std::max(std::max(std::max(boxMax[i], triangle->p1[i]), triangle->p2[i]), triangle->p3[i]);
    }
  }

  std::cout << "Min coordinates: " << boxMin[0] << ", " << boxMin[1] << ", " << boxMin[2] << std::endl;
  std::cout << "Max coordinates: " << boxMax[0] << ", " << boxMax[1] << ", " << boxMax[2] << std::endl;

  root->box = BoundingBox(boxMin, boxMax);
  root->buildTree(root, triangles);
  // to here
  return root;
}

bool KDTree::hasNearestTriangle(Vector<FLOAT,3> eye, Vector<FLOAT,3> direction, Triangle<FLOAT> *  & nearest_triangle, FLOAT &t, FLOAT &u, FLOAT &v, FLOAT minimum_t) {
  // from here
  if (!box.intersects(eye, direction)) return false;

  if (left != nullptr && 
    left->hasNearestTriangle(eye, direction, nearest_triangle, t, u, v, minimum_t) && 
    t < minimum_t) {
    minimum_t = t;
  }

  if (right != nullptr &&
    right->hasNearestTriangle(eye, direction, nearest_triangle, t, u, v, minimum_t) &&
	t < minimum_t) {
    minimum_t = t;
  }

  for (auto const &triangle : this->triangles) {
    stats.no_ray_triangle_intersection_tests++;
    if (triangle->intersects(eye, direction, t, u, v, minimum_t)) {
      stats.no_ray_triangle_intersections_found++;
      if (nearest_triangle == nullptr || t < minimum_t) {
        minimum_t = t;
        nearest_triangle = triangle;
	  }
    }
  }

  t = minimum_t;
  // to here
  return nearest_triangle != nullptr;
}
