from tools import  *
from objects import *
from routines import *


#This file is for strategy

class Gosling(GoslingAgent):
    def run(agent):
        #First thing we do is grab vectors from the ball to all the goalposts
        #We will use these vectors to determine which cars are in the best offense/defense positions
        friend_left = (agent.friend_goal.left_post - agent.ball.location).normalize()
        friend_right = (agent.friend_goal.right_post - agent.ball.location).normalize()
        foe_left = (agent.foe_goal.left_post - agent.ball.location).normalize()
        foe_right = (agent.foe_goal.right_post - agent.ball.location).normalize()

        #Next step collects all sorts of information to evaluate positions and posession
        friend_offense_slopes = sorted([find_slope((agent.ball.location-x.location).clamp(foe_left,foe_right),(agent.ball.location-x.location).normalize()) for x in agent.friends])
        friend_defense_slopes = sorted([find_slope((agent.ball.location-x.location).clamp(friend_left,friend_right),(x.location-agent.ball.location).normalize()) for x in agent.friends])
        my_offense_slope = find_slope((agent.ball.location-agent.me.location).clamp(foe_left,foe_right),(agent.ball.location-agent.me.location).normalize())
        my_defense_slope = find_slope((agent.ball.location-agent.me.location).clamp(friend_left,friend_right),(agent.me.location-agent.ball.location).normalize())
        foe_offense_slopes = sorted([find_slope((agent.ball.location-x.location).clamp(friend_left,friend_right),(agent.ball.location-x.location).normalize()) for x in agent.foes])
        foe_defense_slopes = sorted([find_slope((agent.ball.location-x.location).clamp(foe_left,foe_right),(x.location-agent.ball.location).normalize()) for x in agent.foes])
        best_friend_offense_slope = friend_offense_slopes[-1] if len(friend_offense_slopes) > 0 else -3.0
        best_friend_defense_slope = friend_defense_slopes[-1] if len(friend_defense_slopes) > 0 else -3.0
        best_foe_offense_slope = foe_offense_slopes[-1] if len(foe_offense_slopes) > 0 else -3.0
        best_foe_defense_slope = foe_defense_slopes[-1] if len(foe_defense_slopes) > 0 else -3.0

        friend_approach_times = sorted([(agent.ball.location-friend.location).magnitude()/(2300-cap((agent.ball.location-friend.location).normalize().dot(agent.ball.velocity),-6000,1500)) for friend in agent.friends])
        my_approach_time = cap((agent.ball.location-agent.me.location).magnitude()/(2300-cap((agent.ball.location-agent.me.location).normalize().dot(agent.ball.velocity),-6000,1500)),0,6.0)
        foe_approach_times = sorted([(agent.ball.location-foe.location).magnitude()/(2300-cap((agent.ball.location-foe.location).normalize().dot(agent.ball.velocity),-6000,1500)) for foe in agent.foes])
        best_friend_approach_time = cap(friend_approach_times[0],0,6.0) if len(friend_approach_times) > 0 else 6.0
        best_foe_approach_time = cap(foe_approach_times[0],0,6.0) if len(foe_approach_times) > 0 else 6.0
        
        #Making boolean flags from our new numbers
        have_boost = agent.me.boost > 20
        can_shoot = my_offense_slope > 0.5
        can_defend = my_defense_slope > 0.75
        foe_can_shoot = best_foe_offense_slope > 0.5
        foe_can_defend = best_foe_defense_slope > 0.75
        friend_can_shoot = best_friend_offense_slope > 0.5
        friend_can_defend = best_friend_defense_slope > 0.75
        my_posession = my_approach_time < best_foe_approach_time and my_approach_time < best_friend_approach_time
        foe_posession = best_foe_approach_time < my_approach_time and best_foe_approach_time < best_friend_approach_time
        friend_posession = best_friend_approach_time < my_approach_time and best_friend_approach_time < best_foe_approach_time
        friend_better = best_friend_approach_time < my_approach_time

        close = my_approach_time < 2.0
        foe_close = best_foe_approach_time < 2.0

        action = "nothing"

        if (close or have_boost or not foe_can_defend) and can_defend:
            if friend_better and (friend_can_defend or friend_can_shoot):
                action = "shadow" if have_boost else "grab-boost"
            else:
                action = "goal-shot" if can_shoot else "upfield-shot"
        elif not have_boost and (not foe_can_shoot or friend_can_defend):
            action = "grab-boost"
        elif have_boost:
            if (friend_can_shoot or friend_can_defend) and not can_defend:
                action = "shadow"
            elif close:
                action = "clear"
            else:
                action = "retreat"
        else:
            action = "retreat"

        if len(agent.stack) < 1:
            if agent.kickoff_flag:
                agent.push(kickoff())
            elif action == "goal-shot":
                targets = {"goal":(agent.foe_goal.left_post,agent.foe_goal.right_post)}
                hits = find_hits(agent,targets)
                if len(hits["goal"]) > 0:
                    agent.push(hits["goal"][0])
                else:
                    target = agent.ball.location + ((agent.ball.location-agent.foe_goal.location).normalize() * (agent.ball.location-agent.me.location) / 2)
                    defaultPD(agent,agent.me.local(target-agent.me.location))
                    defaultThrottle(agent,2000)
            elif action == "upfield-shot":
                left_field = Vector3(-4200*side(agent.team),agent.ball.location[1]+(500*side(not agent.team)),92)
                right_field = left_field * Vector3(-1,1,1)
                targets = {"upfield":(left_field,right_field)}
                hits = find_hits(agent,targets)
                if len(hits["upfield"]) > 0:
                    agent.push(hits["upfield"][0])
                else:
                    target = agent.ball.location + ((agent.ball.location-agent.foe_goal.location).normalize() * (agent.ball.location-agent.me.location) / 2)
                    defaultPD(agent,agent.me.local(target-agent.me.location))
                    defaultThrottle(agent,2000)    
            elif action == "clear":
                scaler = Vector3(5,1,1)
                targets = {"clearplus":(agent.foe_goal.right_post*scaler,agent.friend_goal.left_post*scaler),"clearminus":(agent.friend_goal.right_post*scaler,agent.foe_goal.left_post*scaler)}
                hits = find_hits(agent,targets)
                has_plus = len(hits["clearplus"]) > 0
                has_minus = len(hits["clearminus"]) > 0
                if has_plus and has_minus:
                    if hits["clearplus"][0].intercept_time < hits["clearminus"][0].intercept_time:
                        agent.push(hits["clearplus"][0])
                    else:
                        agent.push(hits["clearminus"][0])
                elif has_plus:
                    agent.push(hits["clearplus"][0])
                elif has_minus:
                    agent.push(hits["clearminus"][0])
                else:
                    target = agent.ball.location + ((agent.ball.location-agent.foe_goal.location).normalize() * (agent.ball.location-agent.me.location) / 2)
                    defaultPD(agent,agent.me.local(target-agent.me.location))
                    defaultThrottle(agent,2000)
                    agent.controller.boost = False if agent.me.airborne else agent.controller.boost
            elif action == "short-shot":
                agent.push(short_shot(agent.foe_goal.location))
            elif action == "grab-boost":
                our_boosts = [x for x in agent.boosts if x.large and x.active and -side(agent.team) * x.location.y < (agent.me.location+(agent.me.velocity*1.25)).y]
                if len(our_boosts) < 1:
                    our_boosts = [x for x in agent.boosts if x.active and -side(agent.team) * x.location.y < (agent.me.location+(agent.me.velocity*1.25)).y]
                if len(our_boosts) > 0:
                    closest = our_boosts[0]
                    for boost in our_boosts:
                        if (agent.me.location-boost.location).flatten().magnitude() < (agent.me.location-closest.location).flatten().magnitude():
                            closest = boost
                    agent.push(goto_boost(closest))
            elif action == "retreat":
                defaultPD(agent,agent.me.local(agent.friend_goal.location-agent.me.location))
                defaultThrottle(agent,2300)
                agent.controller.boost = False if agent.me.airborne else agent.controller.boost
            elif action == "shadow":
                direction,distance = (agent.ball.location-agent.friend_goal.location).normalize(True)
                target = agent.friend_goal.location + (direction * (distance/2))
                defaultPD(agent,agent.me.local(target-agent.me.location))
                defaultThrottle(agent,cap(distance*2,200,1410))
                agent.controller.boost = False if agent.me.airborne else agent.controller.boost
            elif action == "nothing":
                pass#print("nothing")
            else:
                print("Action %s not recognized" % action)