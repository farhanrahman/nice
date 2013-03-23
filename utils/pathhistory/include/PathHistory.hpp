#ifndef PATH_HISTORY_HPP
#define PATH_HISTORY_HPP

#include <ros/ros.h>
#include <map>
#include <string>
#include <list>
#include "PathNotFoundException.hpp"

/*#Need to add threadsafety*/
class PathHistory{
public:	
	/*@param[person] person to insert into the pathdb
	 *@param[pathPoint] new pathPoint to insert into list for person
	 **/
	void addPerson(std::string person, tf::StampedTransform pathPoint){
		/*If person already exists then delete the path
		 *history data and add the person as a new entry*/
		if (pathdb.find(person) != pathdb.end()){
			pathdb.erase(person);
		}
		pathdb[person].push_back(pathPoint);
	}

	/*@param[person] lookup path for the person
	 *@throws PathNotFoundException if the path for the person
	 *does not exist*/
	std::list<tf::StampedTransform> getPathHistory(std::string person) throw(PathNotFoundException){
		if (pathdb.find(person) == pathdb.end()) {
			throw PathNotFoundException(person);
		}

		return std::list<tf::StampedTransform> (pathdb[person]);
	}
		
private:
	std::map<Person, std::list<tf::StampedTransform> > pathdb;	
};

#endif
