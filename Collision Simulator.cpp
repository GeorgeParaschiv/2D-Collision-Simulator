/* 
AP Physics Lab 1A 
Collision Simulation
Sarah Ali, George Paraschiv, Arthur Xu
SPH4U0
*/
 
// Define standard mass, radius, time increments
#define M 0.1
#define R 0.05
#define DT 0.0001
 
// Libraries
#include <iostream>
#include <math.h>
#include <string>
#include <fstream>
 
// Using standard namespace
using namespace std;
 
// Vector Class
class Vector
{
public:
    double x, y, m, theta; // Variables for components, magnitude, and angle
 
    Vector() // Default constructor for 0 vector
    {
        x = 0;
        y = 0;
        m = 0;
        theta = 0;
    }
 
    Vector(double a, double b, bool cart) // Vector constructor with user-input components
    {
        if (cart){
            x = a;
            y = b;
            m = sqrt(a * a + b * b); // Calculates magnitude
            theta = atan2(y, x);     // Calculates angle in radians
        }
        else{
            m = a;
            theta = b;
            x = a*cos(b); // Calculates x component
            y = a*sin(b); // Calculates y component
        }
    }

    // Method that checks if the two vectors are parallel
    bool isParallel(Vector v2){
        // Since we are doing a simulation, the y values are often not true zeroes but instead are something like 10e-20
        // Even if the values are not true zeroes, we still consider the vectors as parallel
        // The less than 1e-10 is used to make sure that the value should be a true zero 
        if(abs(y/x-v2.y/v2.x) < pow(1,-10)){ 
            return true;
        }
        else{
            return false;
        }
    }
};
 
// Overrides output operator to print vectors in readable format (x,y)
ostream &operator<<(ostream &strm, const Vector &v)
{
    return strm << "(" << to_string(v.x) << ", " << to_string(v.y) << ")";
}
 
// Ball class
class Ball
{
public:
    Vector s, v, a; // Linear kinematics vectors
    double m, r, w, I, alpha, torque; // Mass and rotational variables: angular velocity, rotational inertia, angular acceleration, torque
    double k; // Spring constant
 
    Ball(Vector s0 = Vector(), Vector v0 = Vector(), double m0 = M, double r0 = R, double k0 = 0, double w0 = 0) //constructor for ball that assigns user input or default values
    {
        s = s0;
        v = v0;
        m = m0;
        r = r0;
        w = w0;
        k = k0;
    }
 
    void applyForce(Vector F, Vector compress, double torque) // Applies force and updates motion variables (part of flowchart)
    {
        a = Vector(F.x / m, F.y / m, true); // Calculate acceleration using Newtons 2nd Law: a = F/m
 
        // Calculate rotational inertia for an ellipsoid: I = 0.2M(a^2 + b^2) where a and b are semi-minor axis
        I = 0.2 * m * (pow(R - compress.m, 2) + pow(R, 2));
        // Calculate angular acceleration using alpha = torque / I
        alpha = torque / I;         
        // Updates linear velocity with kinematics equation v = v0 + a * t
        v = Vector(v.x + a.x * DT, v.y + a.y * DT, true);
        // Updates position with kinematics equation s = s0 + v0 * t -> for small values of t
        s = Vector(s.x + v.x * DT, s.y + v.y * DT, true);
        // Updates angular velocity with rotational kinematics equation w = w0 + alpha * t   
        w += alpha * DT;
    }
};
 
// Driver function
int main()
{
    double sx, sy, vx, vy, w, m, r, T, c, k; // Temporary input values
    double torque; // Value for torque
    Vector spring = Vector(); // Spring force vector
    Vector friction = Vector(); // Friction force vector
    Vector forcenet = Vector(); // Net force vector

    // Obtain User Input for intital conditions (part of the flowchart)
    cout << "Enter the x component of ball 1's position: ";
    cin >> sx;
    cout << "Enter the y component of ball 1's position: ";
    cin >> sy;
    cout << "Enter the x component of ball 1's velocity: ";
    cin >> vx;
    cout << "Enter the y component of ball 1's velocity: ";
    cin >> vy;
    cout << "Enter the angular velocity of ball 1: ";
    cin >> w;
    cout << "Enter the spring constant of ball 1: ";
    cin >> k;
    cout << "Enter the mass of ball 1 (0 for default 0.1 kg): ";
    cin >> m;  
    if (m == 0) // Allows user to choose default mass
        m = M;
    
    // Defines first ball using user input
    Ball b1 = Ball(Vector(sx, sy, true), Vector(vx, vy, true), m, r, k, w);
 
    cout << "Enter the x component of ball 2's position: ";
    cin >> sx;
    cout << "Enter the y component of ball 2's position: ";
    cin >> sy;
    cout << "Enter the x component of ball 2's velocity: ";
    cin >> vx;
    cout << "Enter the y component of ball 2's velocity: ";
    cin >> vy;
    cout << "Enter the angular velocity of ball 2: ";
    cin >> w;
    cout << "Enter the spring constant of ball 2: ";
    cin >> k;
    cout << "Enter the mass of ball 1 (0 for default 0.1 kg): ";
    cin >> m;
    if (m == 0) // Allows user to choose default mass
        m = M;
    
    // Defines second ball using user input
    Ball b2 = Ball(Vector(sx, sy, true), Vector(vx, vy, true), m, r, k, w);

    // Coefficient of friction for the collision
    cout << "Enter the coefficient of friction between the two balls: ";
    cin >> c;

    // Time limit of program in seconds
    cout << "Enter the time limit: ";
    cin >> T;
    
    // Creates a csv output file
    ofstream f("Output.csv");
    
    // Header for output file
    f << "Time" << "," << "Ball 1 Position" << "," << "Ball 2 Position" << "," << "Ball 1 Velocity" << "," << "Ball 2 Velocity" << "," << "Ball 1 Angular Velocity" << "," << "Ball 1 Angular Acceleration" << "," << "Ball 2 Angular Velocity" << "," << "Ball 2 Angular Acceleration" << "\n";

    // Loop for simulation during time interval with dt increments 
    // Have we iterated throught the indicated number of steps (part of flowchart)
    for (double t = 0; t < T; t += DT)
    {
        /* Calculates the relative position vector from ball 1 to ball 2.
        Calculates compression vector using radii and relative position vector (part of flowchart) */
        Vector srel = Vector(b1.s.x - b2.s.x, b1.s.y - b2.s.y, true);
        Vector compression ((b1.r+b2.r - srel.m), srel.theta, false);

        double keff = ((b1.k*b2.k)/(b1.k+b2.k));

        if (compression.m > 0) // Checks if the balls are in contact (part of flowchart)
        {
            // Calculates spring force using F = kx (part of flowchart)
            spring = Vector(compression.x * keff, compression.y * keff, true);
            /* Since the force of friction is perpidicular to the spring force, then the spring force is just the normal force.
            If they are perpindicular, their x and y coordinates are simply switched, with a negative x coordinate in the friction vector.
            Note that we must also multiply by friction coefficient because Force of friction = (coefficient of friction)*(normal force). (part of flowchart)*/
            friction = Vector(-spring.y*c, spring.x*c, true);
            
            /* torque = (radius)x(net tangent force)
                      = (radius)x(force of friction)
            The spring force is always radial and does not apply any torque. (part of flowchart) */
            torque = (R-compression.m)*cos(compression.theta)*friction.y-(R-compression.m)*sin(compression.theta)*friction.x; // This is the cross product definition in 2D

            /* If friction and the translational velocity are perpindicular, there is no frictional force that affects translational velocity, because no component of translational velocity is parallel to friction.
            By definition, friction opposes velocity, but this is not possible if they are perpindicular.
            Therefore friction is not taken into consideration into the net force.
            Since the relative position is perpendicular to the friction, then if velocity is parallel to the relative position, the tranlsational velocity will then be perpendicular to the friction force (part of flowchart)*/
            if(srel.isParallel(b1.v) == true){
                // If no friction force than by default net force is spring force (part of flowchart)
                forcenet = spring;
            }
            else{ //If friction is present the following is true (part of flowchart)
                // Net force adds the spring force to the friction force         
                forcenet = Vector(spring.x + friction.x, spring.y + friction.y, true);                
            }
        }
        else
        {
            // Resets values to 0 if balls are not in contact
            forcenet = Vector();
            torque = 0;
            compression = Vector();
        }

        // Applies force and updates motion variables (part of flowchart)
        b1.applyForce(forcenet, compression, torque);
        b2.applyForce(Vector(-forcenet.x, -forcenet.y, true), compression, -torque);

        // Outputs values to csv file (part of flowchart)
        f << to_string(t) << "," << b1.s << "," << b2.s << "," << b1.v << "," << b2.v << "," << b1.w << "," << b1.alpha << "," << b2.w << "," << b2.alpha << "\n";
    }

    f.close(); // Closes the output stream
}