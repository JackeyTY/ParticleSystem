#ifndef _BEHAVIOR_H
#define _BEHAVIOR_H


// #include "buildsettings.h"
#include <string>
#include "aVector.h"
#include "aJoint.h"
#include "aSkeleton.h"
#include "aActor.h"

using namespace std;

class BehaviorController; 
class AActor;

// Behavior is an abstract base class for all behaviors
class Behavior
{
public:
	Behavior();
    Behavior( Behavior& orig);
    virtual ~Behavior() {}
    virtual  string& GetName() ;

    // Given an actor and behavior parameters, return a desired velocity in world coordinates
    virtual vec3 calcDesiredVel( BehaviorController* actor) = 0;
	virtual AJoint* getTarget() { return m_pTarget; }
	virtual void setTarget(AJoint* target) { m_pTarget = target; }

protected:
    Behavior( char* name);
    string m_name;
	vec3 m_actorPos;
	vec3 m_actorVel;
	AJoint* m_pTarget;
};

// Derived classes: Seek, Flee, Arrival, Departure, Avoid, Wander, Alignment, Separation, Cohesion, Flocking, Leader
///////////////////////////////////////////////////////////////////////////////
class Seek : public Behavior
{
public:
    Seek( AJoint* target);
    Seek( Seek& orig);
    virtual ~Seek();

    virtual vec3 calcDesiredVel( BehaviorController* actor);

protected:

};

///////////////////////////////////////////////////////////////////////////////
class Flee : public Behavior
{
public:
    Flee( AJoint* target);
    Flee( Flee& orig);
    virtual ~Flee();

    virtual vec3 calcDesiredVel( BehaviorController* actor);
	
protected:

};

///////////////////////////////////////////////////////////////////////////////
class Arrival : public Behavior
{
public:
	Arrival( AJoint* target);
	Arrival( Arrival& orig);
	virtual ~Arrival();

	virtual vec3 calcDesiredVel( BehaviorController* actor);

protected:

};


///////////////////////////////////////////////////////////////////////////////
class Departure : public Behavior
{
public:
	Departure( AJoint* target);
	Departure( Departure& orig);
	virtual ~Departure();

	virtual vec3 calcDesiredVel( BehaviorController* actor);

protected:

};


///////////////////////////////////////////////////////////////////////////////
class Obstacle
{
public:
	Obstacle() {}
	virtual ~Obstacle() {}
	double m_Radius;
	AJoint m_Center;
};


///////////////////////////////////////////////////////////////////////////////
class Avoid : public Behavior
{
public:
    Avoid(AJoint* target, vector<Obstacle>* obstacles);
    Avoid( Avoid& orig);
    virtual ~Avoid();

    virtual vec3 calcDesiredVel( BehaviorController* actor);
	vec3 m_obstaclePos;
protected:

    vector<Obstacle>* mObstacles;
    //AJoint& m_Target;
	
};

///////////////////////////////////////////////////////////////////////////////
class Wander : public Behavior
{
public:
	Wander();
	Wander( Wander& orig);
	virtual ~Wander();

	virtual vec3 calcDesiredVel( BehaviorController* actor);

public:
	// the current direction
	vec3 m_Wander;
};

///////////////////////////////////////////////////////////////////////////////
class Alignment : public Behavior
{
public:
	Alignment(AJoint* target, vector<AActor>* agents);
	Alignment( Alignment& orig);
	virtual ~Alignment();

	virtual vec3 calcDesiredVel( BehaviorController* actor);

protected:
	vector<AActor>* m_pAgentList;
	// AJoint& m_Target;
};

///////////////////////////////////////////////////////////////////////////////
class Separation : public Behavior
{
public:
	Separation( AJoint* target,  vector<AActor>* agents);
    Separation( Separation& orig);
    virtual ~Separation();

    virtual vec3 calcDesiredVel( BehaviorController* actor);

protected:
    vector<AActor>* m_AgentList;
    //AJoint& m_Target;
};

///////////////////////////////////////////////////////////////////////////////
class Cohesion : public Behavior
{
public:
    Cohesion( vector<AActor>* agents);
    Cohesion( Cohesion& orig);
    virtual ~Cohesion();

    virtual vec3 calcDesiredVel( BehaviorController* actor);

protected:
    vector<AActor>* m_AgentList;
};

///////////////////////////////////////////////////////////////////////////////
class Flocking : public Behavior
{
public:
    Flocking( AJoint* target,  vector<AActor>* agents);
    Flocking( Flocking& orig);
    virtual ~Flocking();

    virtual vec3 calcDesiredVel( BehaviorController* actor);

protected:
    //AJoint& m_Target;
    vector<AActor>* m_AgentList;
};

///////////////////////////////////////////////////////////////////////////////
class Leader : public Behavior
{
public:
    Leader( AJoint* target, vector<AActor>* agents);
    Leader( Leader& orig);
    virtual ~Leader();

    virtual vec3 calcDesiredVel( BehaviorController* actor);

protected:
    vector<AActor>* m_AgentList;
    //AJoint& m_Target;
};

#endif