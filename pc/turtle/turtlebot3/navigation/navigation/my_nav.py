from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import TaskResult
from geometry_msgs.msg import PoseWithCovarianceStamped
import time
from rclpy.node import Node
import threading
from std_msgs.msg import Int32
from std_msgs.msg import String
from rclpy.duration import Duration
from nav2_simple_commander.robot_navigator import TaskResult
import mysql.connector


class FindLocation(Node):
    def __init__(self):
        super().__init__('navigation')
        
        self.callbot = self.create_subscription(String, 'callbot', self.Callbot, 10 ) # sql 받은 후 엘리베이터 앞 까지 이동
        self.callbot
        self.goelvae = self.create_subscription(String, 'goelvae', self.GoElvaetor, 10) # 엘베 앞으로 이동 (버튼 앞)
        self.goelvae
        self.takein = self.create_subscription(String, 'takein', self.Takein, 10 ) # 엘베 탑승
        self.takein
        self.move2arm = self.create_subscription(String, 'move2arm', self.Move2arm, 10 ) # 현관 앞 주행
        self.move2arm        
        self.littlego = self.create_subscription(String, 'littlego', self.LittleGo, 10 ) # 인증샷
        self.littlego
        self.gobu = self.create_subscription(String, 'gobu', self.Gobu, 10 ) # 엘베 앞으로 다시 이동
        self.gobu       
        self.getin = self.create_subscription(String, 'getin', self.Getin, 10 ) # 탑승
        self.getin      
        self.gohome = self.create_subscription(String, 'gohome', self.Gohome, 10 ) 
        self.gohome        

        
        self.get_logger().info("Waiting command")
        # gazebo 연결 설정
        self.nav = BasicNavigator()
        self.nav.waitUntilNav2Active()
        # 좌표 설정
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
        # 결과값 설정
        self.result = self.nav.getResult()
        
        
    def Callbot(self, msg): # sql 받은 후 엘리베이터 앞 까지 이동

        self.msg_data=msg.data
        
        self.goto_station()
        self.navigate_to_goal()
                
        demo_msg = String()
        demo_msg.data = "getsee"
        demo_publisher = self.create_publisher(String, 'getsee', 10)
        demo_publisher.publish(demo_msg)
            
        
    def GoElvaetor(self, msg): # 엘베 앞으로 이동 (버튼 앞)
        
        self.msg_data=msg.data

        self.sami_point()
        self.navigate_to_goal()
        
        self.first_point()
        self.navigate_to_goal()
        
        self.apart_front()
        self.navigate_to_goal()
        
        self.elevator_front()
        self.navigate_to_goal()
                
        demo_msg = String()
        demo_msg.data = "uparm"
        demo_publisher = self.create_publisher(String, 'uparm', 10)
        demo_publisher.publish(demo_msg)


                        
    def Takein(self, msg): # 엘베 탑승
        
        self.msg_data=msg.data
            
        self.goto_elevator()
        self.navigate_to_goal()
                
        demo_msg = String()
        demo_msg.data = "turnarm"
        demo_publisher = self.create_publisher(String, 'turnarm', 10)
        demo_publisher.publish(demo_msg)

    #sql로 호수라인 확인 후 조건문 실행
    
    def Move2arm(self, msg): # 현관 앞 주행
        
        self.msg_data=msg.data
               
        mydb = mysql.connector.connect(
            host = "final-database.chobs2hlpex2.ap-northeast-2.rds.amazonaws.com",
            port = 3306,
            user = "admin",
            password = "12345678",
            database = "delivery"
        )
        cur = mydb.cursor(buffered=True)
        cur.execute("select name, floor, line from address")

        result = cur.fetchall()

        line = str(result[1]).replace('(', '').replace(')', '').replace("'", "").split(', ')[2]
        
        mydb.close()
        
        if line == "1":
            self.corridor()
            self.navigate_to_goal()
            
            self.corridor_right_room()
            self.navigate_to_goal()
            
            self.right_room()
            self.navigate_to_goal()   
        elif line == "2":       
  
            self.corridor_left_room()
            self.navigate_to_goal()         

            self.left_room()
            self.navigate_to_goal()  
        
        
        demo_msg = String()
        demo_msg.data = "seeho"
        demo_publisher = self.create_publisher(String, 'seeho', 10)
        demo_publisher.publish(demo_msg)           
 
    # sql로 호수 라인 확인 조건문 오른쪽 왼쪽                            
    def LittleGo(self, msg):
        
        mydb = mysql.connector.connect(
            host = "final-database.chobs2hlpex2.ap-northeast-2.rds.amazonaws.com",
            port = 3306,
            user = "admin",
            password = "12345678",
            database = "delivery"
        )
        cur = mydb.cursor(buffered=True)
        cur.execute("select name, floor, line from address")

        result = cur.fetchall()

        line = str(result[1]).replace('(', '').replace(')', '').replace("'", "").split(', ')[2]
        
        mydb.close()
        
        if line == "1":
            #오른쪽 집
            self.right_back()
            self.navigate_to_goal()

            self.right_camear()
            self.navigate_to_goal()
        
        elif line == "2":                   
            #왼쪽 집
            self.left_camear()
            self.navigate_to_goal()

        self.msg_data=msg.data        
        demo_msg = String()
        demo_msg.data = "kimchi"
        demo_publisher = self.create_publisher(String, 'kimchi', 10)
        demo_publisher.publish(demo_msg)             
        
    def Gobu(self, msg):
        
        self.msg_data=msg.data

        self.elevator_front()
        self.navigate_to_goal()
                
        demo_msg = String()
        demo_msg.data = "gobu"
        demo_publisher = self.create_publisher(String, 'seebu', 10)
        demo_publisher.publish(demo_msg)
            
    def Getin(self, msg): #탑승
        
        self.msg_data=msg.data
            
        self.goto_elevator()
        self.navigate_to_goal()
                
        demo_msg = String()
        demo_msg.data = "getin"
        demo_publisher = self.create_publisher(String, 'seele', 10)
        demo_publisher.publish(demo_msg)      
               
    def Gohome(self, msg):
        
        self.msg_data=msg.data
          
            
        self.corridor()
        self.navigate_to_goal()

        self.exit_apart()
        self.navigate_to_goal()
        
        self.final_point()
        self.navigate_to_goal()           
        
        self.goto_station()
        self.navigate_to_goal()
        
    def sami_point(self):
        self.goal_pose.pose.position.x = 0.6696980391876493
        self.goal_pose.pose.position.y = -0.11468513039591005
        self.goal_pose.pose.position.z = 0.0
        self.goal_pose.pose.orientation.x = 0.0
        self.goal_pose.pose.orientation.y = 0.0
        self.goal_pose.pose.orientation.z = 0.05626186737875349
        self.goal_pose.pose.orientation.w = 0.9984160466854765
           
    def first_point(self):
        self.goal_pose.pose.position.x = 1.6213330196042137
        self.goal_pose.pose.position.y = -0.254338175055442
        self.goal_pose.pose.position.z = 0.0
        self.goal_pose.pose.orientation.x = 0.0
        self.goal_pose.pose.orientation.y = 0.0
        self.goal_pose.pose.orientation.z = -0.7423172017849221
        self.goal_pose.pose.orientation.w = 0.670048634006669
    
    
    def apart_front(self):
        self.goal_pose.pose.position.x = 1.654480027905894
        self.goal_pose.pose.position.y = -1.5243505265876103
        self.goal_pose.pose.position.z = 0.0
        self.goal_pose.pose.orientation.x = 0.0
        self.goal_pose.pose.orientation.y = 0.0
        self.goal_pose.pose.orientation.z = -0.7273319349512425
        self.goal_pose.pose.orientation.w = 0.6862858416141787

    def elevator_front(self):
        self.goal_pose.pose.position.x = 1.0043749913566136
        self.goal_pose.pose.position.y = -2.300646292763948
        self.goal_pose.pose.position.z = 0.0
        self.goal_pose.pose.orientation.x = 0.0
        self.goal_pose.pose.orientation.y = 0.0
        self.goal_pose.pose.orientation.z = 0.8636180260190275
        self.goal_pose.pose.orientation.w = 0.5041467099317404
        
    def goto_elevator(self):
        self.goal_pose.pose.position.x = 0.5336671358741386
        self.goal_pose.pose.position.y = -1.755960438602954
        self.goal_pose.pose.position.z = 0.0
        self.goal_pose.pose.orientation.x = 0.0
        self.goal_pose.pose.orientation.y = 0.0
        self.goal_pose.pose.orientation.z = -0.019940952805274404
        self.goal_pose.pose.orientation.w = 0.9998011594318231
        
    def corridor(self):
        self.goal_pose.pose.position.x = 1.1020560441093104
        self.goal_pose.pose.position.y = -1.6944726983878986
        self.goal_pose.pose.position.z = -0.001434326171875
        self.goal_pose.pose.orientation.x = 0.0
        self.goal_pose.pose.orientation.y = 0.0
        self.goal_pose.pose.orientation.z = 0.04295228100931841
        self.goal_pose.pose.orientation.w = 0.9990771249288498

    def corridor_right_room(self):
        self.goal_pose.pose.position.x = 1.0549875486117768
        self.goal_pose.pose.position.y = -1.1290878456531759
        self.goal_pose.pose.position.z = 0.0
        self.goal_pose.pose.orientation.x = 0.0
        self.goal_pose.pose.orientation.y = 0.0
        self.goal_pose.pose.orientation.z = 0.7675223002183216
        self.goal_pose.pose.orientation.w = 0.6410222450645349 

    def corridor_left_room(self):
        self.goal_pose.pose.position.x = 1.0842044110011364
        self.goal_pose.pose.position.y = -2.521937139374835
        self.goal_pose.pose.position.z = 0.0
        self.goal_pose.pose.orientation.x = 0.0
        self.goal_pose.pose.orientation.y = 0.0
        self.goal_pose.pose.orientation.z = -0.7598187310432942
        self.goal_pose.pose.orientation.w = 0.6501349828733708 
        
        
    def right_room(self):
        self.goal_pose.pose.position.x = 0.5886883611739929
        self.goal_pose.pose.position.y = -1.0053328659560918
        self.goal_pose.pose.position.z = 0.0
        self.goal_pose.pose.orientation.x = 0.0
        self.goal_pose.pose.orientation.y = 0.0
        self.goal_pose.pose.orientation.z = -0.6916002168583613
        self.goal_pose.pose.orientation.w = 0.7222805134028383
        
    def right_back(self):
        self.goal_pose.pose.position.x = 0.9909063042686299
        self.goal_pose.pose.position.y = -0.9473965455178318
        self.goal_pose.pose.position.z = 0.0
        self.goal_pose.pose.orientation.x = 0.0
        self.goal_pose.pose.orientation.y = 0.0
        self.goal_pose.pose.orientation.z = -0.02347747041294558
        self.goal_pose.pose.orientation.w = 0.99972436620451    

    def right_camear(self):
        self.goal_pose.pose.position.x = 1.0295718014935347
        self.goal_pose.pose.position.y = -1.105366412330407
        self.goal_pose.pose.position.z = 0.002532958984375
        self.goal_pose.pose.orientation.x = 0.0
        self.goal_pose.pose.orientation.y = 0.0
        self.goal_pose.pose.orientation.z = -0.5209889886439123
        self.goal_pose.pose.orientation.w = 0.8535633975937543       
    
    def left_room(self):
        self.goal_pose.pose.position.x = 0.6046978192521484
        self.goal_pose.pose.position.y = -2.69089136392
        self.goal_pose.pose.position.z = 0.002532958984375
        self.goal_pose.pose.orientation.x = 0.0
        self.goal_pose.pose.orientation.y = 0.0
        self.goal_pose.pose.orientation.z = -0.6578091785064921
        self.goal_pose.pose.orientation.w = 0.7531846285424405
         

    def left_camear(self):
        self.goal_pose.pose.position.x = 1.014975380272809
        self.goal_pose.pose.position.y = -2.662369112154862
        self.goal_pose.pose.position.z = 0.002532958984375
        self.goal_pose.pose.orientation.x = 0.0
        self.goal_pose.pose.orientation.y = 0.0
        self.goal_pose.pose.orientation.z = 0.5246052212398531
        self.goal_pose.pose.orientation.w = 0.8513456183289397     
    
    def exit_apart(self):
        self.goal_pose.pose.position.x = 1.5828068246280096
        self.goal_pose.pose.position.y = -1.823829637778702
        self.goal_pose.pose.position.z = 0.0
        self.goal_pose.pose.orientation.x = 0.0
        self.goal_pose.pose.orientation.y = 0.0
        self.goal_pose.pose.orientation.z = 0.009890960381136644
        self.goal_pose.pose.orientation.w = 0.9999510832549454
        
    def final_point(self):
        self.goal_pose.pose.position.x = 1.6677738225446805
        self.goal_pose.pose.position.y = -0.03257309811842344
        self.goal_pose.pose.position.z = 0.0
        self.goal_pose.pose.orientation.x = 0.0
        self.goal_pose.pose.orientation.y = 0.0
        self.goal_pose.pose.orientation.z = 0.7103397497917234
        self.goal_pose.pose.orientation.w = 0.7038589630500075
                
    def goto_station(self):
        self.goal_pose.pose.position.x = 0.03785225171791947
        self.goal_pose.pose.position.y = -0.17230346864769974
        self.goal_pose.pose.position.z = 0.0
        self.goal_pose.pose.orientation.x = 0.0
        self.goal_pose.pose.orientation.y = 0.0
        self.goal_pose.pose.orientation.z = -0.6617725669114927
        self.goal_pose.pose.orientation.w = 0.7497046549697914
    
    
        
    def navigate_to_goal(self):
        self.nav.goToPose(self.goal_pose)
        i = 0
        while not self.nav.isTaskComplete():
            i = i + 1
            feedback = self.nav.getFeedback()
            if feedback and i % 5 == 0:
                print('Distance remaining: ' + '{:.2f}'.format(
                    feedback.distance_remaining) + ' meters.')
                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self.nav.cancelNav()
                # Some navigation request change to demo preemption
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=120.0):
                    self.goal_pose.pose.position.x = -3.0
                    self.navigate_to_goal()
        if self.nav.isTaskComplete():
            print("turtlebot3 is Elevator Front location arrived!")
            time.sleep(3)            


def main(args=None):

    rclpy.init(args=args)
    
    find_locate = FindLocation()
    
    rclpy.spin(find_locate)
    
    rclpy.shutdown()
    
if __name__ == '__main__':
    
    main()
