//
// Created by mehdi on 1/2/25.
//

#include <map>
#include <iostream>
#include "hemisphere_core.hpp"

namespace hemisphere
{
    //
    // HemishpereCoverageSweep
    //
    void HemishpereCoverageSweep::setup(double r, hemisphere::coverage::DistributionType t, std::vector<double> g, double /*tr*/, Point center)
    {
        distribution_type = t;
        setRadius(r);
        gaussian_vec = g;
        hemi_center = center;
    }

    std::shared_ptr<geometry_msgs::msg::Point> HemishpereCoverageSweep::do_hemisphereCoverage(nav_msgs::msg::Odometry::SharedPtr odometry,  std::map<int, Neighbor> neighbors_map)
    {
        //std::cout << "neighbors_map size " << neighbors_map.size() << std::endl;
        pts.clear();
        new_pts.clear();
        centroids.clear();
        cells.clear();
        half_edgs.clear();
        diagram_pts.clear();

        for(const auto& n : neighbors_map)
        {
            Point neigh_tra = Point(n.second.pos.pose.pose.position.x - hemi_center.x,
                                    n.second.pos.pose.pose.position.y - hemi_center.y,
                                    n.second.pos.pose.pose.position.z - hemi_center.z);
            Point neigh_rot = neigh_tra.rotate(hemi_angles.z);
            sv::IndexedDirection dir(glm::dvec3(neigh_rot.x, neigh_rot.y, neigh_rot.z) , n.first);

            pts.push_back(dir);
        }

        Point odom_tra = Point(odometry->pose.pose.position.x - hemi_center.x,
                               odometry->pose.pose.position.y - hemi_center.y,
                               odometry->pose.pose.position.z - hemi_center.z);
        Point odom_rot = odom_tra.rotate(hemi_angles.z);
        sv::IndexedDirection current_dir(glm::dvec3(odom_rot.x, odom_rot.y, odom_rot.z), drone_id);
        pts.push_back(current_dir);

        // 1. Voronoi diagram generation
        sphericalVoronoiCore = sv::SphericalVoronoiCore(pts);
        sphericalVoronoiCore.setDebugMode(false);

        while (!sphericalVoronoiCore.isFinished()) {
            sphericalVoronoiCore.step(0.1);
        }

        for (const auto& cell : sphericalVoronoiCore.getCells())
        {
            cells.push_back(cell);
        }

        getCurrentDiagram(cells, diagram_pts);

        // 2. Centroids generation
        if(distribution_type == hemisphere::coverage::DistributionType::DISTRIBUTION_GAUSSIAN) {
            std::cout << "Calculate new gaussian centroid" << std::endl;
            computeNewGaussianCentroid(cells, centroids);
        } else {
            std::cout << "Calculate new geometric centroid" << std::endl;
            computeNewGeometricCentroid(cells, centroids);
        }
        std::vector<glm::dvec3> to_publish;

        getAgentsPosition(glm::dvec3(0.0, 0.0, 0.0), centroids, new_pts);

        std::shared_ptr<glm::dvec3> new_destination_1 = std::make_shared<glm::dvec3>(getNewDestination(new_pts, odometry));

        geometry_msgs::msg::Point nd;
        nd.x = new_destination_1->x;
        nd.y = new_destination_1->y;
        nd.z = new_destination_1->z;
        Point nd_(nd.x, nd.y, nd.z);
        centroid = std::make_shared<geometry_msgs::msg::Point>(nd);
        return centroid;

    }

    void HemishpereCoverageSweep::computeNewGeometricCentroid(std::vector<sv::cell_ptr> cells, std::vector<sv::IndexedDirection>& res_pts)
    {
        res_pts.clear();

        for(sv::cell_ptr c : cells)
        {
            sv::half_edge_ptr current_he = c->halfEdges[0];
            sv::half_edge_ptr starting = current_he;

            Point p_(0,0,0);
            int total = 0;

            int index_he = 0;
            do {
                p_.x += current_he->start->point.position[0];
                p_.y += current_he->start->point.position[1];
                p_.z += current_he->start->point.position[2];
                total++;

                current_he = current_he->next;
                index_he++;
            } while(current_he->next != nullptr && starting != current_he);

            Point p__ = p_ / total;
            geometry_msgs::msg::Point p3;
            glm::dvec3 p((p__.x * radius), (p__.y * radius), (p__.z * radius));

            sv::IndexedDirection item(p, c->agent_index);
            res_pts.push_back(item);
            if(c->agent_index == drone_id)
            {
                //std::cout << "computeNewGeometricCentroid  " << p.x << ", " << p.y << ", " << p.z << std::endl;
            }
        }
    }

    void HemishpereCoverageSweep::computeNewGaussianCentroid(std::vector<sv::cell_ptr> cells, std::vector<sv::IndexedDirection>& res_pts)
    {
        res_pts.clear();

        // Check that the gaussian_vec has at least {x, y, z, sigma}
        if (gaussian_vec.size() < 4) {
            std::cerr << "Gaussian vector does not contain enough parameters." << std::endl;
            return;
        }
        // Gaussian center (in 3D) and its standard deviation.
        glm::dvec3 gaussianMean(gaussian_vec[0], gaussian_vec[1], gaussian_vec[2]);
        double sigma = gaussian_vec[3];  // Interpreted as the standard deviation.

        // A resolution parameter for the subdivision of each spherical triangle.
        int resolution = 20;

        // Process every cell (spherical polygon)
        for (auto cell : cells)
        {
            // --- 1. Extract 3D vertices of the polygon (assumed to lie on the sphere) ---
            std::vector<glm::dvec3> vertices;
            sv::half_edge_ptr he = cell->halfEdges[0];
            sv::half_edge_ptr start = he;
            do {
                glm::dvec3 pt(he->start->point.position[0],
                              he->start->point.position[1],
                              he->start->point.position[2]);
                // Ensure the point is on the sphere.
                pt = glm::normalize(pt) * radius;
                vertices.push_back(pt);
                he = he->next;
            } while (he != nullptr && he != start);

            if (vertices.size() < 3)
                continue;  // Not a valid polygon.


            glm::dvec3 weightedSum(0.0);
            double totalWeight = 0.0;
            for (size_t i = 1; i < vertices.size() - 1; i++) {
                glm::dvec3 A = vertices[0];
                glm::dvec3 B = vertices[i];
                glm::dvec3 C = vertices[i+1];
                std::vector<Triangle3D> subTris = subdivideTriangle(A, B, C, resolution, radius);
                for (const auto &tri : subTris) {
                    glm::dvec3 triCentroid = glm::normalize(tri.A + tri.B + tri.C);
                    double area = sphericalTriangleArea(glm::normalize(tri.A), glm::normalize(tri.B), glm::normalize(tri.C), radius);
                    double dotVal = glm::dot(glm::normalize(triCentroid), glm::normalize(gaussianMean));
                    double theta = acos(glm::clamp(dotVal, -1.0, 1.0)); // in radians
                    double d2 = (radius * theta) * (radius * theta); // squared geodesic distance
                    double pdf = exp(-d2 / (2.0 * sigma * sigma));
                    double weight = pdf * area;
                    weightedSum += weight * triCentroid;
                    totalWeight += weight;
                }
            }
            if (totalWeight <= 0.0)
                continue;

            glm::dvec3 newCentroid = weightedSum / totalWeight;
            newCentroid = glm::normalize(newCentroid) * radius;

            res_pts.push_back(sv::IndexedDirection(newCentroid, cell->agent_index));
        }
    }

    /**
     * Method to compute the spherical area of a triangle on a sphere using L'Huilier's formula.
     * ssumes A, B, C are unit vectors (points on the unit sphere). R is the sphere radius.
     */
    double HemishpereCoverageSweep::sphericalTriangleArea(const glm::dvec3& A, const glm::dvec3& B, const glm::dvec3& C, double R) {
        // Compute the arc lengths (in radians) of the sides of the spherical triangle.
        double a = acos(glm::clamp(glm::dot(B, C), -1.0, 1.0));
        double b = acos(glm::clamp(glm::dot(A, C), -1.0, 1.0));
        double c = acos(glm::clamp(glm::dot(A, B), -1.0, 1.0));
        // Compute the semiperimeter.
        double s = (a + b + c) / 2.0;
        // L'Huilier's formula for the spherical excess.
        double tanTerm = sqrt(tan(s/2.0) * tan((s - a)/2.0) * tan((s - b)/2.0) * tan((s - c)/2.0));
        double excess = 4.0 * atan(tanTerm);
        return excess * R * R;
    }

    /**
     * Method to subdivide a spherical triangle (with vertices on a sphere of given radius) into smaller sub-triangles.
     * 'resolution' determines the number of subdivisions along each edge. We create a grid of points using barycentric coordinates.
     * grid[i][j] corresponds to barycentrics: (alpha, beta, gamma) where alpha = (resolution - i - j)/resolution, beta = i/resolution, gamma = j/resolution.
     */
    std::vector<Triangle3D> HemishpereCoverageSweep::subdivideTriangle(const glm::dvec3 &A, const glm::dvec3 &B, const glm::dvec3 &C,
                                              int resolution, double radius) {
        std::vector<Triangle3D> subTriangles;

        std::vector<std::vector<glm::dvec3>> grid(resolution + 1);
        for (int i = 0; i <= resolution; ++i) {
            grid[i].resize(resolution - i + 1);
            for (int j = 0; j <= resolution - i; ++j) {
                double alpha = double(resolution - i - j) / resolution;
                double beta  = double(i) / resolution;
                double gamma = double(j) / resolution;
                // Interpolate in 3D and project back onto the sphere.
                glm::dvec3 point = glm::normalize(alpha * A + beta * B + gamma * C) * radius;
                grid[i][j] = point;
            }
        }
        for (int i = 0; i < resolution; ++i) {
            for (int j = 0; j < resolution - i; ++j) {
                // The current cell corners:
                glm::dvec3 p0 = grid[i][j];
                glm::dvec3 p1 = grid[i+1][j];
                glm::dvec3 p2 = grid[i][j+1];
                subTriangles.push_back({p0, p1, p2});
                if (j < resolution - i - 1) {
                    glm::dvec3 p3 = grid[i+1][j+1];
                    subTriangles.push_back({p1, p3, p2});
                }
            }
        }
        return subTriangles;
    }

    void HemishpereCoverageSweep::getCurrentDiagram(std::vector<sv::cell_ptr> cells, std::vector<Point>& diagram)
    {
        for(sv::cell_ptr c : cells)
        {
            sv::half_edge_ptr current_he = c->halfEdges[0];
            sv::half_edge_ptr starting = current_he;

            int i = 0;
            do {
                // std::cout << "agent_index: " << c->agent_index << " drone_id " << drone_id  << " cells " << cells.size() << std::endl;
                if(c->agent_index == (uint32_t)drone_id) {
                    Point p (current_he->start->point.position[0],
                          current_he->start->point.position[1],
                          current_he->start->point.position[2]);
                    diagram.push_back(p);
                    //std::cout << "getCurrentDiagram index: " << i << p.x << ", " << p.x << ", " << p.z << std::endl;
                }
                current_he = current_he->next;
                i++;

            } while(current_he->next != nullptr && starting != current_he);

            if(diagram.size() != 0)
                diagram.push_back(diagram.at(0));
        }

    }

    void HemishpereCoverageSweep::getAgentsPosition(glm::dvec3 center, std::vector<sv::IndexedDirection> start_pts, std::vector<sv::IndexedDirection>& res_pts)
    {
        res_pts.clear();

        for(sv::IndexedDirection pt : start_pts)
        {
            // Calculate the vector from the center to the point P
            double vx = pt.direction.x - center.x;
            double vy = pt.direction.y - center.y;
            double vz = pt.direction.z - center.z;

            // Calculate the magnitude of the vector
            double magnitude = std::sqrt(vx * vx + vy * vy + vz * vz);

            // Normalize the vector
            double nx = vx / magnitude;
            double ny = vy / magnitude;
            double nz = vz / magnitude;

            double x_ = center.x + nx * radius;
            double y_ = center.y + ny * radius;
            double z_ = center.z + nz * radius;

            // Hemisphere solution is here, take always z>0 points
            //double x__, y__, z__;
            Point point_res;
            if(z_ > hemi_center.z) {
                Point pt = Point(x_, y_, z_);
                //Point pt_rot = pt.rotate(-hemi_angles.z);
                Point pt_rot = pt.rotate(0.0);
                Point pt_tra = Point(pt_rot.x + hemi_center.x, pt_rot.y + hemi_center.y, pt_rot.z + hemi_center.z);
                point_res = pt_tra;
            } else {
                double x__, y__, z__;
                double r = sqrt(x_ * x_ + y_ * y_);
                x__ = ((radius * x_) / r);
                y__ = ((radius * y_) / r);
                z__ = 0.0;
                Point pt = Point(x__, y__, z__);
                //Point pt_rot = pt.rotate(-hemi_angles.z);
                Point pt_rot = pt.rotate(0.0);
                Point pt_tra = Point(pt_rot.x + hemi_center.x, pt_rot.y + hemi_center.y, pt_rot.z + hemi_center.z);
                point_res = pt_tra;
            }
            // Scale the normalized vector to the sphere's radius
            glm::dvec3 surfacePoint(point_res.x, point_res.y, point_res.z);
            res_pts.push_back(sv::IndexedDirection(surfacePoint, pt.index));
            if(pt.index == drone_id)
            {
                //std::cout << "getAgentsPosition  " << surfacePoint.x << ", " << surfacePoint.y << ", " << surfacePoint.z << std::endl;
            }
        }
    }

    glm::dvec3 HemishpereCoverageSweep::getNewDestination(std::vector<sv::IndexedDirection>& centroids, std::shared_ptr<nav_msgs::msg::Odometry> odom)
    {
        glm::dvec3 odom_position(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z);
        glm::dvec3 _centroid;

        for (const auto& centroid : centroids) {
            if(drone_id == centroid.index)
                _centroid = centroid.direction;
        }
        return _centroid;
    }
}
