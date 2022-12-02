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
    Vector second_vector = points[2] - points[1];

    Angle first_angle = Angle::fromRadians( acos(first_vector.dot(second_vector)/(first_vector.length()*second_vector.length()))) - Angle::fromDegrees(90);
    Vector to_first_points = second_vector - first_vector.normalize(second_vector.length()*std::sin(first_angle.toRadians()));
    Point first_point = points[0] - to_first_points.normalize(ROBOT_MAX_RADIUS_METERS) - first_vector.normalize(ROBOT_MAX_RADIUS_METERS); 
    Point second_point = points[0] + to_first_points.normalize(ROBOT_MAX_RADIUS_METERS) - first_vector.normalize(ROBOT_MAX_RADIUS_METERS); 
   
    polygon_points.emplace_back(first_point);
    polygon_points.emplace_back(second_point);

    for(int i = 1; i < ((int)(points.size() - 1)); i++)
    {
        Vector initial_vector = points[i] - points[i-1]; 
        Vector final_vector = points[i+1] - points[i];
        
        Angle inner = Angle::fromRadians( acos(initial_vector.dot(final_vector)/(initial_vector.length()*final_vector.length())) / 2.0 );
        double distance = ROBOT_MAX_RADIUS_METERS / sin(inner.toRadians()); 
        Vector to_point = final_vector.normalize(initial_vector.length()) - initial_vector; 
        Point inner_point = points[i] + to_point.normalize(distance); 

        polygon_points.emplace_back(inner_point);
    }

    Vector last_vector = points[points.size()-1] - points[points.size() - 2];
    Vector before_last_vector = points[points.size() - 2] - points[points.size() - 3];

    Angle last_angle = Angle::fromRadians( acos(before_last_vector.dot(last_vector)/(before_last_vector.length()*last_vector.length()))) - Angle::fromDegrees(90);
    Vector to_last_points = before_last_vector - last_vector.normalize(before_last_vector.length()*std::sin(last_angle.toRadians()));
    Point before_last_point = points[points.size()-1] - to_last_points.normalize(ROBOT_MAX_RADIUS_METERS) - first_vector.normalize(ROBOT_MAX_RADIUS_METERS); 
    Point last_point = points[points.size()-1] + to_last_points.normalize(ROBOT_MAX_RADIUS_METERS) - first_vector.normalize(ROBOT_MAX_RADIUS_METERS); ;
    
    polygon_points.emplace_back(before_last_point);
    polygon_points.emplace_back(last_point);

    for(int i = ((int)(points.size() - 2)); i > 0 ; i--)
    {
        Vector initial_vector = points[i] - points[i-1]; 
        Vector final_vector = points[i+1] - points[i];
        
        Angle inner = Angle::fromRadians( acos(initial_vector.dot(final_vector)/(initial_vector.length()*final_vector.length())) / 2.0 );
        double distance = ROBOT_MAX_RADIUS_METERS / sin(inner.toRadians()); 
        Vector to_point = final_vector.normalize(initial_vector.length()) - initial_vector; 
        Point outer_point = points[i] + to_point.normalize(distance).rotate(Angle::fromDegrees(180)); 

        polygon_points.emplace_back(outer_point);
    }

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
