#include "Physics.h"
#include <cmath>

void ResolveEdgeCollision(Particle& particle, float radius, float screenWidth, float screenHeight) {
    if (particle.position.x - radius < 0.f) {
        particle.position.x = radius;
        particle.velocity.x = std::abs(particle.velocity.x) * particle.restitution;
    } else if (particle.position.x + radius > screenWidth) {
        particle.position.x = screenWidth - radius;
        particle.velocity.x = -std::abs(particle.velocity.x) * particle.restitution;
    }

    if (particle.position.y - radius < 0.f) {
        particle.position.y = radius;
        particle.velocity.y = std::abs(particle.velocity.y) * particle.restitution;
    } else if (particle.position.y + radius > screenHeight) {
        particle.position.y = screenHeight - radius;
        particle.velocity.y = -std::abs(particle.velocity.y) * particle.restitution;
    }
}

void DrawParticle(sf::RenderWindow& window, const Particle& particle, float radius, sf::Color color) {
    sf::CircleShape shape(radius);
    shape.setOrigin({ radius, radius });
    shape.setPosition(particle.position);
    shape.setFillColor(color);
    window.draw(shape);
}
