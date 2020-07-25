

namespace TebPlanner
{
/**
 * @brief computeCenter:compute the vertexs' center in 2D
 * @param vertexs: a lists of vertex
 * @return the center position
 */
Eigen::Vector2d computeCenter(const std::Vector<Eigen::Vector2d>& vertexs);
/*Collision check for differen type*/
/**
 * @brief checkCollisionPoints:check collision between two points
 * @param point1
 * @param point2
 * @param minDistance:the minimum distance to avoid collision
 * @return True if collision occur, otherwise false
 */
bool checkCollisionPoints(const Eigen::Vector2d& point1,const Eigen::Vector2d& point2,const double minDistance);
/**
 * @brief checkCollisionPointLine:check collision between point and line
 * @param point
 * @param line
 * @param minDistance:the minimum distance to avoid collision
 * @return True if collision occur, otherwise false
 */
bool checkCollisionPointLine(const Eigen::Vector2d& point,const std::Vector<Eigen::Vector2d>& line,double minDistance);
/**
 * @brief checkCollisionPointCircle:check collision between point and circle
 * @param point
 * @param center
 * @param radius
 * @param minDistance:the minimum distance to avoid collision
 * @return True if collision occur, otherwise false
 */
bool checkCollisionPointCircle(const Eigen::Vector2d& point,const Eigen::Vector2d& center,
                               double radius,double minDistance);
/**
 * @brief checkCollisionPointPolygon:check collision between point and polygon
 * @param point
 * @param polygon
 * @param minDistance:the minimum distance to avoid collision
 * @return True if collision occur, otherwise false
 */
bool checkCollisionPointPolygon(const Eigen::Vector2d& point,const std::Vector<Eigen::Vector2d>& polygon,double minDistance);
/**
 * @brief checkCollisionLineCircle:check collision between line and circle
 * @param line
 * @param center
 * @param radius
 * @param minDistance:the minimum distance to avoid collision
 * @return True if collision occur, otherwise false
 */
bool checkCollisionLineCircle(const std::Vector<Eigen::Vector2d>& line, const Eigen::Vector2d& center,
                              double radius,double minDistance);
/**
 * @brief checkCollisionLines:check collision between lines
 * @param line1
 * @param line2
 * @param minDistance:the minimum distance to avoid collision
 * @return True if collision occur, otherwise false
 */
bool checkCollisionLines(const std::Vector<Eigen::Vector2d>& line1,const std::Vector<Eigen::Vector2d>& line2,double minDistance);
/**
 * @brief checkCollisionLinePolygon:check collision between line and polygon
 * @param line
 * @param Polygon
 * @param minDistance:the minimum distance to avoid collision
 * @return True if collision occur, otherwise false
 */
bool checkCollisionLinePolygon(const std::Vector<Eigen::Vector2d>& line,const std::Vector<Eigen::Vector2d>& Polygon,double minDistance);
/**
 * @brief checkCollisionCircles:check collision between circles
 * @param center1
 * @param radius1
 * @param center2
 * @param radius2
 * @param minDistance:the minimum distance to avoid collision
 * @return True if collision occur, otherwise false
 */
bool checkCollisionCircles(const Eigen::Vector2d& center1,double radius1,
                           const Eigen::Vector2d& center2,double radius2,double minDistance);
/**
 * @brief checkCollisionCirclePolygon:check collision between circle and polygon
 * @param center
 * @param radius
 * @param polygon
 * @param minDistance:the minimum distance to avoid collision
 * @return True if collision occur, otherwise false
 */
bool checkCollisionCirclePolygon(const Eigen::Vector2d& center,double radius,const std::Vector<Eigen::Vector2d>& polygon,double minDistance);
/**
 * @brief checkCollisionPolygons:check collision between polygons
 * @param polygon1
 * @param polygon2
 * @param minDistance:the minimum distance to avoid collision
 * @return True if collision occur, otherwise false
 */
bool checkCollisionPolygons(const std::Vector<Eigen::Vector2d>& polygon1,const std::Vector<Eigen::Vector2d>& polygon2,double minDistance);
/*minDistance compute for differen type*/
/**
 * @brief computeDistancePoints:compute distance between two points
 * @param point1
 * @param point2
 * @return the minimum distance
 */
double computeDistancePoints(const Eigen::Vector2d& point1,const Eigen::Vector2d& point2);
/**
 * @brief computeDistancePointLine:compute distance between point and line
 * @param point
 * @param line
 * @return the minimum distance
 */
double computeDistancePointLine(const Eigen::Vector2d& point,const std::Vector<Eigen::Vector2d>& line);
/**
 * @brief computeDistancePointCircle:compute distance between point and circle
 * @param point
 * @param center
 * @param radius
 * @return the minimum distance
 */
double computeDistancePointCircle(const Eigen::Vector2d& point,const Eigen::Vector2d& center,
                               double radius);
/**
 * @brief computeDistancePointPolygon:compute distance between point and polygon
 * @param point
 * @param polygon
 * @return the minimum distance
 */
double computeDistancePointPolygon(const Eigen::Vector2d& point,const std::Vector<Eigen::Vector2d>& polygon);
/**
 * @brief computeDistanceLineCircle:compute distance between line and circle
 * @param line
 * @param center
 * @param radius
 * @return the minimum distance
 */
double computeDistanceLineCircle(const std::Vector<Eigen::Vector2d>& line, const Eigen::Vector2d& center,
                              double radius);
/**
 * @brief computeDistanceLines:compute distance between lines
 * @param line1
 * @param line2
 * @return the minimum distance
 */
double computeDistanceLines(const std::Vector<Eigen::Vector2d>& line1,const std::Vector<Eigen::Vector2d>& line2);
/**
 * @brief computeDistanceLinePolygon:compute distance between line and polygon
 * @param line
 * @param Polygon
 * @return the minimum distance
 */
double computeDistanceLinePolygon(const std::Vector<Eigen::Vector2d>& line,const std::Vector<Eigen::Vector2d>& Polygon);
/**
 * @brief computeDistanceCircles:compute distance between circles
 * @param center1
 * @param radius1
 * @param center2
 * @param radius2
 * @return the minimum distance
 */
double computeDistanceCircles(const Eigen::Vector2d& center1,double radius1,
                           const Eigen::Vector2d& center2,double radius2);
/**
 * @brief computeDistanceCirclePolygon:compute distance between circle and polygon
 * @param center
 * @param radius
 * @param polygon
 * @return the minimum distance
 */
double computeDistanceCirclePolygon(const Eigen::Vector2d& center,double radius,const std::Vector<Eigen::Vector2d>& polygon);
/**
 * @brief computeDistancePolygons:compute distance between polygons
 * @param polygon1
 * @param polygon2
 * @return the minimum distance
 */
double computeDistancePolygons(const std::Vector<Eigen::Vector2d>& polygon1,const std::Vector<Eigen::Vector2d>& polygon2);

namespace LossFunction
{
double rangeBound(double value,double upper_limit,double lower_limit,double epsilon=0.0);
double lowerBound(double value,double lower_limit,double epsilon=0.0);
double upperBound(double value,double upper_limit,double epsilon=0.0);
}
namespace MathFunction
{
double fastSigmoid(double value);
}
}
