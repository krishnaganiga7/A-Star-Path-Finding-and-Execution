#!/usr/bin/env python

import rospy
import numpy as np
from math import atan2
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

def scancb(msg):
	global ls
	ls=msg.ranges[0:359]


import numpy as np
import math

def positcb(msg):
	global x
	global y
	global theta
	global phi
	x=msg.pose.pose.position.x
	y=msg.pose.pose.position.y
	ori=msg.pose.pose.orientation
	(alpha,beta,theta)=euler_from_quaternion([ori.x,ori.y,ori.z,ori.w])

	
class Node():
    
    def __init__(self,parent=None,position=None):
        
        self.parent=parent
        self.position=position
        self.g=0
        self.h=0
        
        self.f=0

    def __eq__(self,other):
        return self.position==other.position
        
global start
global end
        
def astar():

    global mpath
    maz = [0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,
       0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,
       0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,
       0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,
       0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,1,0,
       0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1,
       0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1,
       0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1,
       0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0,
       0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,0,
       0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,0,
       0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0,
       0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1]
    
    a=np.array(maz)
    maz=np.reshape(a,(20,18))
    vis=maz
    start=(11,0)
    startNode=Node(None,start)
    end=(0,13)
    endNode=Node(None,end)
    
    openl=[]
    closedl=[]
    openl.append(startNode)
    
    while(len(openl)!=0):
        mini=float('inf')
        for i in openl:
            print(i.position,i.f)
            if(i.f<mini):
                

                mini=i.f
                cur=i
        print("The length of open list",len(openl))
        print("The CURRENT NODE is",cur.position)
        #print("This is openl",openl)

        openl.remove(cur)
        closedl.append(cur.position)
        
        if(cur.position==end):
            i=cur
            print(i.position)
            mpath=[]
            
            while(i.position!=(11,0)):
                i=i.parent
                vis[i.position]=5
                print(i.position)
                mpath.append(i.position)

            mpath.reverse()
            mpath.append(end)
            print(vis)
            print(mpath)
            return 0
        
        for i in [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]:
            #print ("FUUUU")
            #print("we're in child",i)
            cp=(i[0]+cur.position[0],i[1]+cur.position[1])
            print("The child is",cp, "and it was a obstacle" ,maz[cp])
            if (cp[0]<0) or (cp[0]>len(maz)-1) or (cp[1]<0) or (cp[1]>len(maz[0])-1) or (cp in closedl) or (maz[cp]==1):
                print("Bad child")
                continue

            xx=0
            #print("the openl is",openl)
            for n in openl:
                if n.position==cp :
                    #print("the math value",math.sqrt(i[0]**2+i[1]**2))
                    f=cur.g+math.sqrt(i[0]**2+i[1]**2)
                    #print("Theeeeee")
                    if(f<n.g):
                        n.g=f
                        n.parent=cur
                        print("The g and parent of ",np,"was updated to ",f,cur.parent)
                        xx=1
                        
            if(xx==0):

                if(i[0]==1 and i[1]==1):
                    print("gooooo")
                    #print("The left ele index ",cp[0]-1,cp[1])
                    #print("The down ele index is",cp[0],cp[1]-1)
                    #le=(cp[0]-1,cp[1])
                    #de=(cp[0],cp[1]-1)
                    #print("The left and down maze obtsacle",maz[le],maz[de])
                    
                    if((maz[(cp[0]-1,cp[1])]==1) and (maz[cp[0],cp[1]-1]==1)):
                        print("diagonal obstacle")
                        continue
                if(i[0]==-1 and i[1]==-1):
                    print("Herrr")
                    if((maz[cp[0]+1,cp[1]]==1) and (maz[cp[0],cp[1]+1]==1)):
                        print("diagonal obstacle")
                        continue
                if(i[0]==-1 and i[1]==1):
                    print("Yayyy")
                    if((maz[cp[0]+1,cp[1]]==1) and (maz[cp[0],cp[1]-1]==1)):
                        print("diagonal obstacle")
                        continue
            
                if(i[0]==1 and i[1]==-1):
                    print("Yayyy")
                    if((maz[cp[0]-1,cp[1]]==1) and (maz[cp[0],cp[1]+1]==1)):
                        print("diagonal obstacle")
                        continue


                tl=Node(cur,cp)
                tl.g=cur.g+math.sqrt(i[0]**2+i[1]**2)
                #print("The g of",cp," is",tl.g)
                #print(end[0],cp[0],end[1],cp[1])
                tl.h=math.sqrt((end[0]-cp[0])**2+ (end[1]-cp[1])**2)
                #print("The h of",cp," is",tl.h)
                tl.f=tl.g+tl.h

                print("The f of",cp,"is",tl.f)
                
                if(tl not in openl):
                    openl.append(tl)
                #print("The length of openl is",len(openl))

def postom(pos):
    
	r=int(9-pos[1])
	c=int(8+pos[0])
	return r,c


print(postom((4.5,9)))


def maptop(r,c):
    
    
    pos=[]
    pos.append(c-8.5)
    pos.append(9-r)
    return pos

#print(maptop(12.02,7.98))

if __name__ == '__main__':

	mpath=[]
	thres=36
	ls=0
	print("Hi!")
	rospy.init_node('alternate',anonymous=True)
	rospy.Rate(10)
	pub=rospy.Publisher('cmd_vel',Twist,queue_size=10)
	ho=Twist()
	rospy.Subscriber('base_scan',LaserScan,scancb)
	#rospy.sleep(1)
	print(ls)
	astar()
	path=[]
	for i in mpath:
		path.append(maptop(i[0],i[1]))

	#print("The simulator path is ",path)
	ls = np.reshape(ls, (thres,-1))



