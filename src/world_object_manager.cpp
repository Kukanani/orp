#include "orp/core/world_object_manager.h"
#include "orp/core/world_object.h"

WorldObjectManager::WorldObjectManager(std::string unknownName) :
	unknownType(WorldObjectType(unknownName))
{
	types.insert(std::pair<std::string, WorldObjectType>(unknownName, unknownType));
} //WorldObjectManager

void WorldObjectManager::addType(WorldObjectType wot)
{
	types[wot.name] = wot;
} //addType

WorldObjectType& WorldObjectManager::getTypeByName(std::string name)
{
	std::map<std::string, WorldObjectType>::iterator found = types.find(name);
	if(found != types.end()) {
		return found->second;
	}
	throw std::logic_error("did not find object named '" + name + "'");
	return unknownType;
} //getTypeByName
