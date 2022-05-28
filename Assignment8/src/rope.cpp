#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.

		for (int i = 0; i < num_nodes; i++)
		{
			auto mass = new Mass(start + (end - start) * i / num_nodes, node_mass, false);
			masses.push_back(mass);
		}

		for (int i = 1; i < num_nodes; i++)
		{
			auto spring = new Spring(masses[i - 1], masses[i], k);
			springs.push_back(spring);
		}

        for (auto &i : pinned_nodes)
		{
            masses[i]->pinned = true;
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
			auto dir = s->m2->position - s->m1->position;
			auto force = s->k * dir.unit() * (dir.norm() - s->rest_length);
			s->m1->forces += force;
			s->m2->forces -= force;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
				m->velocity += (m->forces / m->mass + gravity) * delta_t;
				m->position += m->velocity * delta_t;
                // TODO (Part 2): Add global damping

            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
			auto dir = s->m2->position - s->m1->position;
			auto force = s->k * dir.unit() * (dir.norm() - s->rest_length);
			s->m1->forces += force;
			s->m2->forces -= force;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 3.1): Set the new position of the rope mass
				// TODO (Part 4): Add global Verlet damping
				static constexpr double damping_factor = 5e-8;
				Vector2D tmp_position = m->position;
				m->position += (1 - damping_factor) * (m->position - m->last_position) + (m->forces / m->mass + gravity) * delta_t * delta_t;
				m->last_position = tmp_position;
            }

			// Reset all forces on each mass
			m->forces = Vector2D(0, 0);
        }
    }
}
