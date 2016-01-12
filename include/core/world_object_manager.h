#ifndef _WORLD_OBJECT_MANAGER_H_
#define _WORLD_OBJECT_MANAGER_H_

#include <map>
#include <stdexcept>

#include <core/world_object.h>
/**
 * @brief Keeps track of world object types so that their properties remain constant.
 *
 * @version 1.0
 * @ingroup objectrecognition
 * @ingroup apc
 * 
 * @author    Adam Allevato <allevato@utexas.edu>
 * @copyright BSD 3-paragraph
 * @date      1/29/2015
 */
class WorldObjectManager {
private:
	WorldObjectType unknownType; ///Default object type
public:
	std::map<std::string, WorldObjectType> types; //The possible world object types
	
	/**
	 * Create a new world object manager with the specified default type.
	 * @arg unknown the world object to fall back on when nothing can be found.
	 */
	WorldObjectManager(std::string unknown);


	/// Return how many object types this manager is aware of.
	int getNumTypes() { return types.size(); };

	/// Return how many object types this manager is aware of.
	WorldObjectType getUnknownType() { return unknownType; };

	///Add an object type to the type list.
	void addType(WorldObjectType wot);

	///Retrieve a type by name, or the "unknown" object type if not found.
	WorldObjectType& getTypeByName(std::string name);
}; //WorldObjectManager

#endif // _WORLD_OBJECT_MANAGER_H_