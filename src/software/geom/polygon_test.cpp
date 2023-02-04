#include "software/geom/polygon.h"

#include <gtest/gtest.h>

#include <unordered_set>
#include "software/ai/navigator/obstacle/obstacle.hpp"
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
    Point p1{0.0f, 0.0f}, p2{1.0f, 0.0f}, p3{0.0f, 1.0f}, p4{1.0f, 1.0f},
          p5{3.0f,0.0f}, p6{2.0f,1.0f};

    // In the order below, these four points will create a downward facing U shape. 
    std::vector<Point> test_down_u_pts; 
    test_down_u_pts.emplace_back(p1); 
    test_down_u_pts.emplace_back(p3);
    test_down_u_pts.emplace_back(p4);
    test_down_u_pts.emplace_back(p2); 
    Polygon test_down_u = Polygon::fromMultiplePoints(test_down_u_pts);

    // These are the points expected to be just contained by the polygon at every joint or end
    Point t1{-ROBOT_MAX_RADIUS_METERS+0.01, -ROBOT_MAX_RADIUS_METERS+0.01},
          t2{ROBOT_MAX_RADIUS_METERS-0.01,-ROBOT_MAX_RADIUS_METERS+0.01},
          t3{-ROBOT_MAX_RADIUS_METERS+0.01,0.99+ROBOT_MAX_RADIUS_METERS},
          t4{ROBOT_MAX_RADIUS_METERS-0.01,1.01-ROBOT_MAX_RADIUS_METERS},
          t5{1.01-ROBOT_MAX_RADIUS_METERS,1.01-ROBOT_MAX_RADIUS_METERS},
          t6{0.99+ROBOT_MAX_RADIUS_METERS,0.99+ROBOT_MAX_RADIUS_METERS},
          t7{1.01-ROBOT_MAX_RADIUS_METERS,-ROBOT_MAX_RADIUS_METERS+0.01},
          t8{0.99+ROBOT_MAX_RADIUS_METERS,-ROBOT_MAX_RADIUS_METERS+0.01};

    // These are the points expected to be just not contained by the polygon at every joint or end
    Point f1{-ROBOT_MAX_RADIUS_METERS-0.01, -ROBOT_MAX_RADIUS_METERS-0.01},
          f2{ROBOT_MAX_RADIUS_METERS+0.01,-ROBOT_MAX_RADIUS_METERS-0.01},
          f3{-ROBOT_MAX_RADIUS_METERS-0.01,1.01+ROBOT_MAX_RADIUS_METERS},
          f4{ROBOT_MAX_RADIUS_METERS+0.01,0.99-ROBOT_MAX_RADIUS_METERS},
          f5{0.99-ROBOT_MAX_RADIUS_METERS,0.99-ROBOT_MAX_RADIUS_METERS},
          f6{1.01+ROBOT_MAX_RADIUS_METERS,1.01+ROBOT_MAX_RADIUS_METERS},
          f7{0.99-ROBOT_MAX_RADIUS_METERS,-ROBOT_MAX_RADIUS_METERS-0.01},
          f8{1.01+ROBOT_MAX_RADIUS_METERS,-ROBOT_MAX_RADIUS_METERS-0.01};

    EXPECT_TRUE(contains(test_down_u, t1));
    EXPECT_TRUE(contains(test_down_u,t2));
    EXPECT_TRUE(contains(test_down_u,t3));
    EXPECT_TRUE(contains(test_down_u,t4));
    EXPECT_TRUE(contains(test_down_u,t5));
    EXPECT_TRUE(contains(test_down_u,t6));
    EXPECT_TRUE(contains(test_down_u,t7));
    EXPECT_TRUE(contains(test_down_u,t8));

    EXPECT_FALSE(contains(test_down_u,f1));
    EXPECT_FALSE(contains(test_down_u,f2));
    EXPECT_FALSE(contains(test_down_u,f3));
    EXPECT_FALSE(contains(test_down_u,f4));
    EXPECT_FALSE(contains(test_down_u,f5));
    EXPECT_FALSE(contains(test_down_u,f6));
    EXPECT_FALSE(contains(test_down_u,f7));
    EXPECT_FALSE(contains(test_down_u,f8));

    // In the order below, these four points will create a slanted u shape: \_\ ;
    std::vector<Point> test_slant_u_pts; 
    test_slant_u_pts.emplace_back(p3); 
    test_slant_u_pts.emplace_back(p2);
    test_slant_u_pts.emplace_back(p5);
    test_slant_u_pts.emplace_back(p6); 
    Polygon test_slant_u = Polygon::fromMultiplePoints(test_slant_u_pts);

    // These are the points expected to be just contained by the polygon at every joint of end
    Point g1{-sqrt(2)*ROBOT_MAX_RADIUS_METERS+0.01,1};

    EXPECT_TRUE(contains(test_slant_u,g1));

}