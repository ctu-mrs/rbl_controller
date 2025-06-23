// process_pointcloud.h
#ifndef PROCESS_POINTCLOUD_H
#define PROCESS_POINTCLOUD_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/gp3.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/centroid.h>
#include <pcl/common/geometry.h>
#include <pcl/conversions.h>

#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/PolygonMesh.h>

#include <random>
#include <algorithm>
#include <queue>
#include <Eigen/Dense>
#include <Eigen/Geometry>


// #include <pcl/surface/marching_cubes_hho.h>
#include <string>
#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>

class PointCloudProcessor {
public:
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsamplePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float leafSize, Eigen::Vector3d robot_pos, double encumbrance, double radius_sensing) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;

        voxel_grid.setInputCloud(cloud);
        voxel_grid.setLeafSize(leafSize, leafSize, leafSize);
        voxel_grid.filter(*cloud_downsampled);

        pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
        filteredCloud->points.reserve(cloud_downsampled->points.size());

        for (const auto& point : cloud_downsampled->points) {
            Eigen::Vector3d p(point.x, point.y, point.z);
            double distance = (p - robot_pos).norm();

            if (distance > encumbrance && distance < radius_sensing) {
                filteredCloud->points.push_back(point);
            }
        }

        filteredCloud->width = filteredCloud->points.size();
        filteredCloud->height = 1;
        filteredCloud->is_dense = true; // no Nans should be in pcl

        return filteredCloud;
    }

    pcl::PolygonMesh createSingleTriangleMesh(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2, const pcl::PointXYZ& p3) {
        // This is just test function delete later  
        pcl::PolygonMesh mesh;
        pcl::PointCloud<pcl::PointXYZ> cloud;

        // 1. Create the Vertices
        cloud.push_back(p1);
        cloud.push_back(p2);
        cloud.push_back(p3);

        // 2. Create the Polygon
        pcl::Vertices triangle;
        triangle.vertices.push_back(0);
        triangle.vertices.push_back(1);
        triangle.vertices.push_back(2);

        mesh.polygons.push_back(triangle);

        // 3. Set the Header
        mesh.cloud.width = cloud.size();
        mesh.cloud.height = 1;
        mesh.cloud.is_dense = true;
        pcl::toPCLPointCloud2(cloud, mesh.cloud);

        return mesh;
    }

    pcl::PolygonMesh createMesh(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float max_dist) {
        pcl::PolygonMesh mesh;

        if (cloud->size()==0) {
          return mesh;
        }

        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(cloud);

        std::vector<pcl::Vertices> polygons;

        //1. Generate potential triangles 
        for (size_t i = 0; i < cloud->size(); ++i) {
            pcl::PointXYZ searchPoint = cloud->points[i];
            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;

            if (kdtree.radiusSearch(searchPoint, max_dist, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
                for (size_t j = 0; j < pointIdxRadiusSearch.size(); ++j) {
                    if (pointIdxRadiusSearch[j] > static_cast<int>(i)) { // Avoid duplicate triangles
                        pcl::Vertices polygon;
                        polygon.vertices.push_back(i);
                        polygon.vertices.push_back(pointIdxRadiusSearch[j]);

                        // Find the third point for a triangle
                        for (size_t k = j + 1; k < pointIdxRadiusSearch.size(); ++k) {
                            if (pointIdxRadiusSearch[k] > static_cast<int>(i)) {
                                polygon.vertices.push_back(pointIdxRadiusSearch[k]);
                                if(polygon.vertices.size() == 3){
                                    polygons.push_back(polygon);
                                    polygon.vertices.pop_back();
                                }
                            }
                        }
                    }
                }
            }
        }


        pcl::toPCLPointCloud2(*cloud, mesh.cloud);
        mesh.polygons = polygons;

        return mesh;
    }

    pcl::PolygonMesh createConvexHullMesh(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
        pcl::PolygonMesh mesh;
    
        // Create a Convex Hull object
        pcl::ConvexHull<pcl::PointXYZ> chull;
        chull.setInputCloud(cloud);
        chull.reconstruct(mesh);
    
        return mesh;
    }

    pcl::PolygonMesh createGP3Mesh(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double searchRadius, double mu) {
        // auto start_time = std::chrono::high_resolution_clock::now(); // Start timing
        // Normal estimation*
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
        pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud (cloud);
        n.setInputCloud (cloud);
        n.setSearchMethod (tree);
        n.setKSearch (20);
        n.compute (*normals);
        //* normals should not contain the point normals + surface curvatures

        // Concatenate the XYZ and normal fields*
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
        pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
        //* cloud_with_normals = cloud + normals

        // Create search tree*
        pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
        tree2->setInputCloud (cloud_with_normals);

        // // Initialize objects
        pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
        pcl::PolygonMesh triangles;

        // maximum edge length
        gp3.setSearchRadius (searchRadius);

        // Set typical values for the parameters
        gp3.setMu(mu);
        gp3.setMaximumNearestNeighbors(75);
        gp3.setMaximumSurfaceAngle(2*M_PI/3); 
        gp3.setMinimumAngle(M_PI/18); 
        gp3.setMaximumAngle(3*M_PI/4); 
        gp3.setNormalConsistency(false);

        // // Get result
        gp3.setInputCloud (cloud_with_normals);
        gp3.setSearchMethod (tree2);
        gp3.reconstruct (triangles);

        // float tolerance = 0.05; // Maximum distance from point to plane
        // int min_vertices = 10; // Minimum vertices for a plane proxy

        // auto start_time = std::chrono::high_resolution_clock::now(); // Start timing

        // pcl::PolygonMesh simplified_mesh = planarSimplification(triangles);

        // auto end_time = std::chrono::high_resolution_clock::now(); // End timing

        // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time); // Calculate duration

        // std::cout << "create gp3 mesh took " << duration.count() << " milliseconds." << std::endl;
        // std::cout << "planarSimplification took " << duration.count() << " milliseconds." << std::endl;
        return triangles;
    }


    pcl::PolygonMesh planarSimplification(const pcl::PolygonMesh& mesh) {
      //Computationaly expensive, maybe use for actual mapping the mesh? - do not have to recompute all the time
        pcl::PolygonMesh simplified_mesh;
        std::vector<pcl::Vertices> triangles = mesh.polygons;
        
        if (triangles.size()<=0) {
            return simplified_mesh;
        }
        int n_seeds = triangles.size()/30;
        
        std::vector<int> seedIndices = getSeedIndices(n_seeds, triangles.size());

        std::vector<std::vector<uint32_t>> polygons;
        std::vector<bool> triangleUsed(triangles.size(), false);
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromPCLPointCloud2(mesh.cloud, cloud);
        float normalTolerance = 0.1f;
        
        std::vector<pcl::Vertices> simplified_polygons;
        

        for (int seedIndex : seedIndices) {
            if (triangleUsed[seedIndex]) continue; // Skip if already used or seed.
            triangleUsed[seedIndex] = true;
    
            std::vector<uint32_t> currentPolygonVertices;
            std::queue<int> triangleQueue;
            triangleQueue.push(seedIndex);
    
            Eigen::Vector3f seedNormal = calculateTriangleNormal(triangles[seedIndex], cloud);
    
            while (!triangleQueue.empty()) {
                int currentIndex = triangleQueue.front();
                triangleQueue.pop();
    
                pcl::Vertices& currentTriangle = triangles[currentIndex];
                currentPolygonVertices.insert(currentPolygonVertices.end(), currentTriangle.vertices.begin(), currentTriangle.vertices.end());
    
                std::vector<int> neighborIndices = getNeighborIndices(currentIndex, triangles, cloud);
    
                for (int neighborIndex : neighborIndices) {
                    if (triangleUsed[neighborIndex]) continue;
                    triangleUsed[neighborIndex] = true;
    
                    Eigen::Vector3f neighborNormal = calculateTriangleNormal(triangles[neighborIndex], cloud);
    
                    if (areNormalsAligned(seedNormal, neighborNormal, normalTolerance)) {
                        triangleQueue.push(neighborIndex);
                    }
                }
            }
            // std::cout << "Current polygon vertices " << currentPolygonVertices.size() << std::endl;    
            polygons.push_back(currentPolygonVertices);//todo del

            pcl::PointCloud<pcl::PointXYZ> projectedCloud;
            pcl::PointXYZ& firstPoint = cloud.points[triangles[seedIndex].vertices[0]];
            Eigen::Vector3f planePoint = Eigen::Vector3f(firstPoint.x, firstPoint.y, firstPoint.z);
            for (uint32_t vertexIndex : currentPolygonVertices) {
                pcl::PointXYZ point = projectPointOnPlane(cloud.points[vertexIndex], seedNormal, planePoint);
                cloud.points[vertexIndex] = point;
                projectedCloud.points.push_back(point);
            }

            // std::cout << "Current polygon vertices before simplification: " << currentPolygonVertices.size() << std::endl; 
            createConvexHullPoints(projectedCloud, seedNormal, cloud, currentPolygonVertices);
            // std::cout << "Current polygon vertices after simplification: " << currentPolygonVertices.size() << std::endl;  

            pcl::Vertices simplifiedPolygonVertices;
            simplifiedPolygonVertices.vertices.resize(currentPolygonVertices.size()); // Resize the vector
            for (size_t i = 0; i < currentPolygonVertices.size(); ++i) {
                simplifiedPolygonVertices.vertices[i] = static_cast<int>(currentPolygonVertices[i]); // Convert uint32_t to int
            }
            simplified_polygons.push_back(simplifiedPolygonVertices);
        }
        
        // for (size_t i = 0; i < triangles.size(); ++i) {
        //     if (!triangleUsed[i]) {
        //         simplified_polygons.push_back(triangles[i]);
        //     }
        // }

        pcl::toPCLPointCloud2(cloud, simplified_mesh.cloud); // Copy cloud data
        simplified_mesh.polygons = simplified_polygons;

        return simplified_mesh;
    }


    void createConvexHullPoints(pcl::PointCloud<pcl::PointXYZ>& projectedCloud, 
                                Eigen::Vector3f seedNormal, 
                                pcl::PointCloud<pcl::PointXYZ>& cloud, 
                                std::vector<uint32_t>& currentPolygonVertices) {
        //  just a testing function
        
        if (projectedCloud.empty()) return; // Handle empty cloud case

        rotatePlanarPointsToXY(projectedCloud, seedNormal);

        // Create 2D point cloud for convex hull computation
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& point : projectedCloud.points) {
            pcl::PointXYZ p2d(point.x, point.y, 0); // Z coordinate is 0 for 2D
            cloud_2d->points.push_back(p2d);
        }

        // Compute convex hull
        pcl::PointCloud<pcl::PointXYZ> hull;
        pcl::ConvexHull<pcl::PointXYZ> convex_hull;
        convex_hull.setInputCloud(cloud_2d);
        convex_hull.reconstruct(hull);

        std::vector<uint32_t> hullIndices;
        for (const auto& hullPoint : hull.points) {
            for (size_t i = 0; i < projectedCloud.size(); ++i) {
                if (std::abs(projectedCloud[i].x - hullPoint.x) < 1e-2 &&
                    std::abs(projectedCloud[i].y - hullPoint.y) < 1e-2) {
                    hullIndices.push_back(i);
                    break;
                }
            }
        }

        // std::cout << "Current polygon vertices before simplification: " << currentPolygonVertices.size() << std::endl;
        // std::cout << "Found hull vertices: " << hullIndices.size() << std::endl;

        // Update currentPolygonVertices directly using hullIndices
        std::vector<uint32_t> newPolygonVertices;
        for (uint32_t hullIndex : hullIndices) {
            newPolygonVertices.push_back(currentPolygonVertices[hullIndex]);
        }

        currentPolygonVertices = newPolygonVertices;
    }

    void rotatePlanarPointsBack(pcl::PointCloud<pcl::PointXYZ>& projectedCloud, const Eigen::Vector3f& planeNormal) {
        if (projectedCloud.size() < 3) {
            return; // Need at least 3 points to define a plane
        }
    
        Eigen::Vector3f zAxis(0.0f, 0.0f, 1.0f);
        Eigen::Quaternionf rotation = Eigen::Quaternionf::FromTwoVectors(planeNormal, zAxis);
        Eigen::Matrix3f rotationMatrix = rotation.toRotationMatrix();
    
        for (auto& point : projectedCloud) {
            Eigen::Vector3f p(point.x, point.y, point.z);
            Eigen::Vector3f rotatedP = rotationMatrix * p;
            point.x = rotatedP.x();
            point.y = rotatedP.y();
            point.z = rotatedP.z();
        }
    }

    void rotatePlanarPointsToXY(pcl::PointCloud<pcl::PointXYZ>& projectedCloud, const Eigen::Vector3f& planeNormal) {
        if (projectedCloud.size() < 3) {
            return;
        }
    
        Eigen::Vector3f zAxis(0.0f, 0.0f, 1.0f);
        Eigen::Quaternionf rotation = Eigen::Quaternionf::FromTwoVectors(planeNormal, zAxis);
        Eigen::Matrix3f rotationMatrix = rotation.toRotationMatrix();
    
        for (auto& point : projectedCloud) { 
            Eigen::Vector3f p(point.x, point.y, point.z);
            Eigen::Vector3f rotatedP = rotationMatrix * p;
            point.x = rotatedP.x();
            point.y = rotatedP.y();
            point.z = rotatedP.z();
        }
    }

    
    pcl::PointXYZ projectPointOnPlane(const pcl::PointXYZ& point, const Eigen::Vector3f& planeNormal, const Eigen::Vector3f& planePoint) {
        Eigen::Vector3f p = Eigen::Vector3f(point.x, point.y, point.z);
    
        Eigen::Vector3f v = p - planePoint;
    
        float distance = v.dot(planeNormal);
    
        Eigen::Vector3f projected = p - distance * planeNormal;
    
        pcl::PointXYZ projectedPoint;
        projectedPoint.x = projected(0);
        projectedPoint.y = projected(1);
        projectedPoint.z = projected(2);
    
        return projectedPoint;
    }

    Eigen::Vector3f calculateTriangleNormal(const pcl::Vertices& triangle, const pcl::PointCloud<pcl::PointXYZ>& cloud) {
        if (triangle.vertices.size() != 3) return Eigen::Vector3f::Zero(); // Only for triangles
    
        Eigen::Vector3f p1(cloud.points[triangle.vertices[0]].x, cloud.points[triangle.vertices[0]].y, cloud.points[triangle.vertices[0]].z);
        Eigen::Vector3f p2(cloud.points[triangle.vertices[1]].x, cloud.points[triangle.vertices[1]].y, cloud.points[triangle.vertices[1]].z);
        Eigen::Vector3f p3(cloud.points[triangle.vertices[2]].x, cloud.points[triangle.vertices[2]].y, cloud.points[triangle.vertices[2]].z);
    
        Eigen::Vector3f v1 = p2 - p1;
        Eigen::Vector3f v2 = p3 - p1;
    
        return v1.cross(v2).normalized();
    }

    bool areNormalsAligned(const Eigen::Vector3f& n1, const Eigen::Vector3f& n2, float tolerance) {
        float dotProduct = n1.dot(n2);
        return std::abs(dotProduct) > (1.0f - tolerance);
    }
    
    std::vector<int> getNeighborIndices(int triangleIndex, const std::vector<pcl::Vertices>& triangles, const pcl::PointCloud<pcl::PointXYZ>& cloud) {
        std::vector<int> neighbors;
        const pcl::Vertices& currentTriangle = triangles[triangleIndex]; // Changed to const
        for (size_t i = 0; i < triangles.size(); ++i) {
            if ((int)i == triangleIndex) continue;
            const pcl::Vertices& potentialNeighbor = triangles[i]; // Changed to const
            for (uint32_t v1 : currentTriangle.vertices) {
                for (uint32_t v2 : potentialNeighbor.vertices) {
                    if (v1 == v2) {
                        neighbors.push_back(i);
                        goto nextTriangle; // Break out of both inner loops
                    }
                }
            }
            nextTriangle:;
        }
        return neighbors;
    }

    std::vector<int> getSeedIndices(int n_seeds, int n_triangles) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, n_triangles - 1);

        std::vector<int> seedIndices;
        if (n_triangles <= n_seeds){
            for(size_t i = 0; i < n_triangles; ++i){
                seedIndices.push_back(i);
            }
        } else {
            while (seedIndices.size() < n_seeds) {
                int randomIndex = dis(gen);
                if (std::find(seedIndices.begin(), seedIndices.end(), randomIndex) == seedIndices.end()) {
                    seedIndices.push_back(randomIndex);
                }
            }
        }
        return seedIndices;
    }
};



#endif // PROCESS_POINTCLOUD_H
