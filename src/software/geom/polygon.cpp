#include "software/geom/polygon.h"
#include "shared/constants.h"

#include <unordered_set>

Polygon::Polygon(const std::vector<Point>& points)
    : points_(points), segments_(initSegments(points_))
{
    // we pre-compute the segments_ in the constructor to improve performance
}

Polygon::Polygon(const std::initializer_list<Point>& points)
    : Polygon(std::vector(points))
{
}

std::vector<Segment> Polygon::initSegments(std::vector<Point> points)
{
    std::vector<Segment> segments;
    for (unsigned i = 0; i < points.size(); i++)
    {
        // add a segment between consecutive points, but wrap index
        // to draw a segment from the last point to first point.
        segments.emplace_back(Segment{points[i], points[(i + 1) % points.size()]});
    }
    return segments;
}

Point Polygon::centroid() const
{
    // Explanation of the math/geometry behind this:
    // https://fotino.me/calculating-centroids/
    double x_centre    = 0;
    double y_centre    = 0;
    double signed_area = 0;

    for (unsigned i = 0; i < points_.size(); i++)
    {
        double x0 = points_[i].x();
        double y0 = points_[i].y();
        double x1 = points_[(i + 1) % points_.size()].x();
        double y1 = points_[(i + 1) % points_.size()].y();
        double a  = (x0 * y1) - (x1 * y0);

        x_centre += (x0 + x1) * a;
        y_centre += (y0 + y1) * a;
        signed_area += a;
    }

    return Point((Vector(x_centre, y_centre) / (3 * signed_area)));
}

Polygon Polygon::expand(double expansion_amount) const
{
    if (expansion_amount < 0)
    {
        throw std::invalid_argument(
            "Polygon::expand: expansion_amount must be non-negative");
    }
    Point centroid_point = centroid();
    std::vector<Point> expanded_points;
    expanded_points.reserve(points_.size());

    Vector last_expansion =
        (segments_[0].midPoint() - centroid_point).normalize(expansion_amount);
    Point first_point = segments_[0].getStart() + last_expansion;
    for (size_t i = 1; i < segments_.size(); i++)
    {
        Vector current_expansion =
            (segments_[i].midPoint() - centroid_point).normalize(expansion_amount);
        expanded_points.emplace_back(segments_[i].getStart() + current_expansion +
                                     last_expansion);
        last_expansion = current_expansion;
    }
    expanded_points.emplace_back(first_point + last_expansion);

    return Polygon(expanded_points);
}

Polygon Polygon::fromSegment(const Segment& segment, const double radius)
{
    /*   The Polygon is constructed as follows:
     *
     *        start_l                start_r
     *           +----------+----------+
     *           |          |          |
     *           |          | radius   |
     *           |          |          |
     *           +   start  X          +
     *           |          |          |
     *           |          |          |
     *           |          |          |
     *           |       segment       |
     *           |          |          |
     *           |          |          |
     *           |          |   radius |
     *           +   end    X----------+
     *           |                     |
     *           |                     |
     *           |                     |
     *           +----------+----------+
     *         end_l                 end_r
     */

    Vector start_to_end = segment.getEnd().toVector() - segment.getStart().toVector();
    Vector end_to_start = -start_to_end;

    Point end_l = segment.getEnd() + (start_to_end.normalize(radius) -
                                      start_to_end.perpendicular().normalize(radius));
    Point end_r = segment.getEnd() + (start_to_end.normalize(radius) +
                                      start_to_end.perpendicular().normalize(radius));

    Point start_l = segment.getStart() + (end_to_start.normalize(radius) +
                                          end_to_start.perpendicular().normalize(radius));
    Point start_r = segment.getStart() + (end_to_start.normalize(radius) -
                                          end_to_start.perpendicular().normalize(radius));

    return Polygon({
        start_l,
        start_r,
        end_r,
        end_l,
    });
}

Polygon Polygon::fromMultiplePoints(const std::vector<Point>& points)
{
    std::vector<Point> polygon_points; 

    Vector first_vector = points[1] - points[0];
    Point first_point = points[0] + first_vector.rotate(Angle::fromDegrees(90)).normalize(ROBOT_MAX_RADIUS_METERS); 
    Point second_point = points[0] + first_vector.rotate(Angle::fromDegrees(-90)).normalize(ROBOT_MAX_RADIUS_METERS); 

    polygon_points.emplace_back(first_point);
    polygon_points.emplace_back(second_point);

    for(int i = 1; i < ((int)(points.size() - 1)); i++)
    {
        Vector initial_vector = points[i] - points[i-1]; 
        Vector final_vector = points[i+1] - points[i];
        
        Angle ccw = Angle::fromRadians( acos(initial_vector.dot(final_vector)/(initial_vector.length()*final_vector.length())) );
        double ccw_distance = (2*ROBOT_MAX_RADIUS_METERS) / sin(ccw.toRadians() / 2.0); 
        Vector to_first_point = initial_vector.rotate(ccw).normalize(ccw_distance);
        Point ccw_point = points[i] + to_first_point; 

        Angle cw = ccw + Angle::fromDegrees(180); 
        double cw_distance = (2*ROBOT_MAX_RADIUS_METERS) / sin(cw.toRadians() / 2.0);
        Vector to_second_point = initial_vector.rotate(cw).normalize(cw_distance);
        Point cw_point = points[i] + to_second_point; 

        polygon_points.emplace_back(ccw_point);
        polygon_points.emplace_back(cw_point); 
    }

    Vector last_vector = points[points.size()] - points[points.size() - 1];
    Point before_last_point = points[points.size()] + last_vector.rotate(Angle::fromDegrees(90)).normalize(ROBOT_MAX_RADIUS_METERS);
    Point last_point = points[points.size()] + last_vector.rotate(Angle::fromDegrees(-90)).normalize(ROBOT_MAX_RADIUS_METERS);

    polygon_points.emplace_back(before_last_point);
    polygon_points.emplace_back(last_point);

    return Polygon(polygon_points); 
}

const std::vector<Segment>& Polygon::getSegments() const
{
    return segments_;
}

const std::vector<Point>& Polygon::getPoints() const
{
    return points_;
}

bool operator==(const Polygon& poly1, const Polygon& poly2)
{
    return (poly1.getPoints() == poly2.getPoints());
}

bool operator!=(const Polygon& poly1, const Polygon& poly2)
{
    return !(poly1 == poly2);
}

std::ostream& operator<<(std::ostream& os, const Polygon& poly)
{
    os << "Polygon with points {";
    for (const auto& pt : poly.getPoints())
    {
        os << pt << ' ';
    }
    os << '}';
    return os;
}
