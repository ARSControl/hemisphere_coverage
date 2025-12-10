//
// Created by mehdi on 1/2/25.
//

#ifndef BUILD_HEMISPHERE_CORE_HPP
#define BUILD_HEMISPHERE_CORE_HPP
#pragma once

// Msg
#include <nav_msgs/msg/odometry.hpp>

// Standard
#include <cmath>
#include <functional>
#include <tuple>
#include <vector>
#include <map>
#include <memory>
#include <string>

// Spherical voronoi
#include "fortune_algorithm/svVoronoiCore.h"
#include <glm/gtx/string_cast.hpp>

namespace hemisphere
{
    namespace coverage {
        enum class DistributionType
        {
            DISTRIBUTION_GEOMETRICAL = 0,
            DISTRIBUTION_GAUSSIAN = 1
        };
    }
}

namespace hemisphere
{
    struct Point
    {
        double x, y, z;
        Point() { x = 0.0; y = 0.0; z = 0.0; }
        Point(double x_, double y_, double z_) : x(x_), y(y_), z(z_) { }

        Point operator/(int divisor) const {
            return Point(x / divisor, y / divisor, z / divisor);
        }

        Point rotate(double yaw_angle) {
            double c = std::cos(yaw_angle);
            double s = std::sin(yaw_angle);
            return Point(c * x - s * y, s * x + c * y, z);
        }
    };

    struct Neighbor
    {
        int index;
        uint64_t stamp_ns{0};
        nav_msgs::msg::Odometry pos;
        Neighbor() {index = 0; stamp_ns = 0; }
        Neighbor(int index_, uint64_t stamp_ns_, nav_msgs::msg::Odometry pos_) : index(index_), stamp_ns(stamp_ns_), pos(pos_) { }
    };

    struct Triangle3D
    {
        glm::dvec3 A, B, C;
    };

    enum StateMachine
    {
        INIT = 0,
        HEMISHPERE = 1
    };

    using state_func = std::function<void(const StateMachine &)>;
    using point_func = std::function<void(const Point &)>;
    using points_func = std::function<void(const std::vector<Point> &)>;
    using sphere_func = std::function<void(const Point &, const double &, const std::tuple<double,double,double,double> &)>;

    /**
     * Core class with common attributes
     */
    class HemisphereCoverageCore
    {

    public:
        HemisphereCoverageCore(int id, std::string /*config_path*/) { drone_id = id; }

        void setDistributionType(hemisphere::coverage::DistributionType type_) { distribution_type = type_; }
        void setRadius(double r) { radius = r; }
        void setGaussianValues(std::vector<double> g) { gaussian_vec = g; }
        void setCenter(Point center) { hemi_center = center; }
        void setAngles(Point angles) { hemi_angles = angles; }
        virtual void setup(double r, hemisphere::coverage::DistributionType t, std::vector<double> g, double tr, Point center) = 0;

        inline void registerChangeInStateCallback(state_func callback) { state_function = callback; }

        virtual std::shared_ptr<geometry_msgs::msg::Point> do_hemisphereCoverage(nav_msgs::msg::Odometry::SharedPtr odometry,  std::map<int, Neighbor> neighbors_map) = 0;

    protected:
        int                                     drone_id;
        double                                  radius;
        hemisphere::coverage::DistributionType  distribution_type;
        std::vector<double>                     gaussian_vec;
        std::vector<Point>                      diagram_pts;
        std::vector<Point>                      arcs_pts;
        Point                                   hemi_center;
        Point                                   hemi_angles;

        state_func                              state_function;

    };



    /**
     *  @brief HemishpereCoverageSweep -
     */
    class HemishpereCoverageSweep : public HemisphereCoverageCore
    {
    public:
        HemishpereCoverageSweep(int id, std::string config_path) : HemisphereCoverageCore(id, config_path) { }

        void setup(double r, hemisphere::coverage::DistributionType t, std::vector<double> g, double tr, Point center) override;

        std::shared_ptr<geometry_msgs::msg::Point> do_hemisphereCoverage(nav_msgs::msg::Odometry::SharedPtr odometry,  std::map<int, Neighbor> neighbors_map) override;

    private:
        sv::SphericalVoronoiCore sphericalVoronoiCore;
        std::vector<sv::IndexedDirection> pts;
        std::vector<sv::IndexedDirection> new_pts;
        std::vector<sv::IndexedDirection> centroids;
        std::vector<sv::cell_ptr> cells;
        std::vector<sv::half_edge> half_edgs;

        std::shared_ptr<geometry_msgs::msg::Point> centroid;

        void computeNewGeometricCentroid(std::vector<sv::cell_ptr> cells, std::vector<sv::IndexedDirection>& res_pts);
        void computeNewGaussianCentroid(std::vector<sv::cell_ptr> cells, std::vector<sv::IndexedDirection>& res_pts);
        double sphericalTriangleArea(const glm::dvec3& A, const glm::dvec3& B, const glm::dvec3& C, double R);
        std::vector<Triangle3D> subdivideTriangle(const glm::dvec3 &A, const glm::dvec3 &B, const glm::dvec3 &C,
                                                                           int resolution, double radius);
        void getCurrentDiagram(std::vector<sv::cell_ptr> cells, std::vector<Point>& diagram);
        void getAgentsPosition(glm::dvec3 center, std::vector<sv::IndexedDirection> start_pts, std::vector<sv::IndexedDirection>& res_pts);
        glm::dvec3 getNewDestination(std::vector<sv::IndexedDirection>& centroids, std::shared_ptr<nav_msgs::msg::Odometry> odom);

    };
};


#endif //BUILD_HEMISPHERE_CORE_HPP
