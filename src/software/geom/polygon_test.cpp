#include "software/geom/polygon.h"

#include <gtest/gtest.h>

#include <unordered_set>
#include "software/geom/point.h"
#include "software/test_util/test_util.h"

TEST(PolygonTest, test_construct_from_vector)
{
    Point p1{0.0f, 0.0f}, p2{1.0f, 0.0f}, p3{1.0f, 1.0f};
    std::vector<Point> points{p1, p2, p3};
    Polygon poly(points);

    // check that all of the points are in the polygon
    std::unordered_set<Point> points_set{p1, p2, p3};
    for (const auto& p : poly.getPoints())
    {
        EXPECT_TRUE(points_set.find(p) != points_set.end());
    }

    // check that the correct segments are in the polygon
    std::unordered_set<Segment> segments_set = {Segment{p1, p2}, Segment{p2, p3},
                                                Segment{p3, p1}};
    for (const auto& seg : poly.getSegments())
    {
        EXPECT_TRUE(segments_set.find(seg) != segments_set.end());
    }
}

TEST(PolygonTest, test_construct_from_initializer_list)
{
    Point p1{0.0f, 0.0f}, p2{1.0f, 0.0f}, p3{1.0f, 1.0f};
    Polygon poly{p1, p2, p3};

    // check that all of the points are in the polygon
    std::unordered_set<Point> points_set{p1, p2, p3};
    for (const auto& p : poly.getPoints())
    {
        EXPECT_TRUE(points_set.find(p) != points_set.end());
    }

    // check that the correct segments are in the polygon
    std::unordered_set<Segment> segments_set = {Segment{p1, p2}, Segment{p2, p3},
                                                Segment{p3, p1}};
    for (const auto& seg : poly.getSegments())
    {
        EXPECT_TRUE(segments_set.find(seg) != segments_set.end());
    }
}



TEST(PolygonCentroidTest, test_triangle)
{
    Polygon poly({{1, 2}, {2, 3}, {1, 3}});
    EXPECT_TRUE(TestUtil::equalWithinTolerance(Point(1.333, 2.666), poly.centroid(),
                                               METERS_PER_MILLIMETER));
}

TEST(PolygonCentroidTest, test_rectangle)
{
    Polygon poly({{-1, -1}, {-1, 3}, {5, 3}, {5, -1}});
    EXPECT_TRUE(TestUtil::equalWithinTolerance(Point(2, 1), poly.centroid(),
                                               METERS_PER_MILLIMETER));
}

TEST(PolygonCentroidTest, test_irregular_shape)
{
    Polygon poly({{-1, -1}, {-8, 4}, {-2, 4}, {-2, 2}, {2, 0}});
    EXPECT_TRUE(TestUtil::equalWithinTolerance(Point(-2.895, 1.842), poly.centroid(),
                                               METERS_PER_MILLIMETER));
}

TEST(PolygonCentroidTest, test_non_convex_five_points_up_left)
{
    Polygon poly({{1, 1}, {1, 3}, {2, 2}, {5, 3}, {5, 1}});
    EXPECT_TRUE(TestUtil::equalWithinTolerance(Point(3.111, 1.778), poly.centroid(),
                                               METERS_PER_MILLIMETER));
}


TEST(PolygonExpandTest, test_four_points_slanted)
{
    // These points make a slanted 5 unit long square
    Polygon poly({{3, 4}, {7, 1}, {4, -3}, {0, 0}});
    // These points make a slanted 10 unit long square
    Polygon expected({{10.5, 1.5}, {4.5, -6.5}, {-3.5, -0.5}, {2.5, 7.5}});
    // To double the initial square, we need to add 2.5 units in all 4 directions
    EXPECT_EQ(poly.expand(2.5), expected);
}

TEST(PolygonExpandTest, test_invalid_modifier)
{
    Polygon poly({{-1, 1}, {1, 1}, {1, -1}, {-1, -1}});
    Polygon expected({{-2, 2}, {2, 2}, {2, -2}, {-2, -2}});
    try
    {
        EXPECT_EQ(poly.expand(-2), expected);
        GTEST_FAIL();
    }
    catch (const std::invalid_argument& e)
    {
        GTEST_SUCCEED();
    }
}

TEST(PolygonExpandTest, test_from_segments)
{
    const auto segment = Segment(Point(0, 0), Point(2, 2));
    double radius      = 1;

    const auto poly     = Polygon::fromSegment(segment, radius);
    const auto segments = poly.getSegments();

    Vector first_side  = segments[0].toVector();
    Vector second_side = segments[1].toVector();
    Vector third_side  = segments[2].toVector();
    Vector fourth_side = segments[3].toVector();

    Angle ninety_degrees     = Angle::fromDegrees(90);
    double long_side_length  = radius * 2 + segment.length();
    double short_side_length = radius * 2;


    // check side lengths
    EXPECT_TRUE(
        TestUtil::equalWithinTolerance(first_side.length(), short_side_length, 0.001));
    EXPECT_TRUE(
        TestUtil::equalWithinTolerance(second_side.length(), long_side_length, 0.001));
    EXPECT_TRUE(
        TestUtil::equalWithinTolerance(third_side.length(), short_side_length, 0.001));
    EXPECT_TRUE(
        TestUtil::equalWithinTolerance(fourth_side.length(), long_side_length, 0.001));

    // check there's only 4 segments
    EXPECT_EQ(segments.size(), 4);

    // check that angle between all sides is ninety degrees
    EXPECT_EQ(first_side.orientation() - second_side.orientation(), ninety_degrees);
    EXPECT_EQ(second_side.orientation() - third_side.orientation(), ninety_degrees);
    EXPECT_EQ(third_side.orientation() - fourth_side.orientation(), ninety_degrees);
    EXPECT_EQ(fourth_side.orientation() - first_side.orientation(), ninety_degrees);
}

TEST(PolygonfromPointsTest, general_test)
{
    Field field = Field::createSSLDivisionBField();

    std::vector<Point> points; 
    //std::vector<Point> polygon_points; 
            
    points.emplace_back(field.friendlyGoalpostPos());
    points.emplace_back(field.friendlyGoalBackPos());
    points.emplace_back(field.friendlyGoalBackNeg());
    points.emplace_back(field.friendlyGoalpostNeg());

    Vector first_vector = points[1] - points[0];
    Point first_point = points[0] + first_vector.rotate(Angle::fromDegrees(-135)).normalize(std::sqrt(2)*ROBOT_MAX_RADIUS_METERS); 
    Point second_point = points[0] + first_vector.rotate(Angle::fromDegrees(135)).normalize(std::sqrt(2)*ROBOT_MAX_RADIUS_METERS); 
    //polygon_points.emplace_back(first_point);
    std::cout << std::endl << "first point: " << first_point << std::endl;
    //polygon_points.emplace_back(second_point);
    std::cout << "second point: " << second_point << std::endl; 

    for(int i = 1; i < ((int)(points.size() - 1)); i++)
    {
        Vector initial_vector = points[i] - points[i-1]; 
        Vector final_vector = points[i+1] - points[i];
        
        Angle ccw = Angle::fromRadians( - acos(initial_vector.dot(final_vector)/(initial_vector.length()*final_vector.length())) / 2.0 );
        double distance = ROBOT_MAX_RADIUS_METERS / sin(ccw.toRadians()); 
        Vector to_first_point = initial_vector.rotate(ccw).normalize(distance);
        Point ccw_point = points[i] + to_first_point; 

        //polygon_points.emplace_back(ccw_point);
        std::cout << "next point (angle  = " << ccw << ", distance = " << distance << "): " << ccw_point << std::endl;  
    }

    Vector last_vector = points[points.size()-1] - points[points.size() - 2];
    Point before_last_point = points[points.size()-1] + last_vector.rotate(Angle::fromDegrees(-135)).normalize(std::sqrt(2)*ROBOT_MAX_RADIUS_METERS);
    Point last_point = points[points.size()-1] + last_vector.rotate(Angle::fromDegrees(135)).normalize(std::sqrt(2)*ROBOT_MAX_RADIUS_METERS);
    //polygon_points.emplace_back(before_last_point);
    std::cout << "before last point: " << before_last_point << std::endl; 
    //polygon_points.emplace_back(last_point);
    std::cout << "last point: " << last_point << std::endl; 

    for(int i = ((int)(points.size() - 2)); i > 0 ; i--)
    {
        Vector initial_vector = points[i] - points[i-1]; 
        Vector final_vector = points[i+1] - points[i];
        
        Angle ccw = Angle::fromRadians( - acos(initial_vector.dot(final_vector)/(initial_vector.length()*final_vector.length())) / 2.0 );
        Angle cw = ccw + Angle::fromDegrees(180);
        double distance = ROBOT_MAX_RADIUS_METERS / sin(ccw.toRadians()); 
        Vector to_first_point = initial_vector.rotate(cw).normalize(distance);
        Point cw_point = points[i] + to_first_point; 

        //polygon_points.emplace_back(cw_point);
        std::cout << "next point (angle  = " << cw << ", distance = " << distance << "): " << cw_point << std::endl;   

    }

    std::cout << std::endl << std::endl << "size implementation test" << std::endl << "size: " << points.size() - 1 << ", that point = " << points[points.size() - 1];
}