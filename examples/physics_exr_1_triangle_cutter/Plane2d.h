#include <SFML/Graphics.hpp>

class Plane2d
{
public:
    sf::Vector2f point;
    sf::Vector2f normal;

    float signedDistance(const sf::Vector2f& p) const
    {
        return (p - point).dot(normal);
    }
};