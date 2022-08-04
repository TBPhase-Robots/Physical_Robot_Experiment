# backup code for the as yet unimplemented shuffle function

if(state == "shuffle"):
    # get all robots and their end positions

    # if a robot is in an end node that is not theirs
    global robotsInTheWay
    robotsInTheWay = []

    sheepPositions = cfg['initial_sheep_positions']
    dogPositions = cfg['initial_dog_positions']
    standbyPositions = cfg['standby_positions']
    pigPositions = cfg['pigsty_positions']
    # add all non sheep agents who are in the way of sheep destinations to robotsInTheWay
    robotsInTheWay.append(pathfindingManager.GetAgentsInTheWay(targetPositions=sheepPositions, agentRole='sheep', agents = agents))

    # add all non dog agents who are in the way of sheep destinations to robotsInTheWay
    robotsInTheWay.append(pathfindingManager.GetAgentsInTheWay(targetPositions=sheepPositions, agentRole='dog', agents = agents))

    # add all non standby agents who are in the way of sheep destinations to robotsInTheWay
    robotsInTheWay.append(pathfindingManager.GetAgentsInTheWay(targetPositions=sheepPositions, agentRole='standby', agents = agents))

    # add all non standby agents who are in the way of sheep destinations to robotsInTheWay
    robotsInTheWay.append(pathfindingManager.GetAgentsInTheWay(targetPositions=sheepPositions, agentRole='pigs', agents = agents))
    
    flat_list = []
    for sublist in robotsInTheWay:
        for item in sublist:
            flat_list.append(item)
            
    stationaryAgents = []
    # for every agent, if it is not in flat list, add it to stationary
    for agent in agents:
        agentId = agent.id
        fail = False
        for robot in flat_list:
            if(agentId == robot.id):
                fail = True
        if(not fail):
            stationaryAgents.append(agent)



    # endpoints in screen coordinates
    endpoints = []
    # endpoints in pathfinding coordinates
    endpointTiles = []
    # for each agent in robotsInTheWay, calculate a nearby pathfinding tile not occupied

    pathfindingManager.GenerateWorldMatrix(stationaryAgents=stationaryAgents, endPoints=endpoints)
    for agent in stationaryAgents:
        print("finding available position for stationary agent ", agent.id)
        # send currently taken endpoint tiles, 
        pathfindingManager
        # return endpoints and endpointTile


    
    # for each agent in flat list, give them the nearest non occupied space to follow