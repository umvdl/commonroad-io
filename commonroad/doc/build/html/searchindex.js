Search.setIndex({docnames:["api/common","api/geometry","api/index","api/planning","api/prediction","api/scenario","api/visualization","index","user/getting_started","user/index","user/visualization"],envversion:53,filenames:["api/common.rst","api/geometry.rst","api/index.rst","api/planning.rst","api/prediction.rst","api/scenario.rst","api/visualization.rst","index.rst","user/getting_started.rst","user/index.rst","user/visualization.rst"],objects:{"commonroad.common":{file_reader:[0,0,0,"-"],file_writer:[0,0,0,"-"],solution_writer:[0,0,0,"-"],util:[0,0,0,"-"]},"commonroad.common.file_reader":{CommonRoadFileReader:[0,1,1,""]},"commonroad.common.file_reader.CommonRoadFileReader":{open:[0,2,1,""],open_lanelet_network:[0,2,1,""]},"commonroad.common.file_writer":{CommonRoadFileWriter:[0,1,1,""],OverwriteExistingFile:[0,1,1,""]},"commonroad.common.file_writer.CommonRoadFileWriter":{check_validity_of_commonroad_file:[0,3,1,""],write_scenario_to_file:[0,2,1,""],write_scenario_to_file_io:[0,2,1,""],write_to_file:[0,2,1,""],write_to_file_io:[0,2,1,""]},"commonroad.common.file_writer.OverwriteExistingFile":{ALWAYS:[0,4,1,""],ASK_USER_INPUT:[0,4,1,""],SKIP:[0,4,1,""]},"commonroad.common.solution_writer":{CommonRoadSolutionWriter:[0,1,1,""],CostFunction:[0,1,1,""],VehicleModel:[0,1,1,""],VehicleType:[0,1,1,""]},"commonroad.common.solution_writer.CommonRoadSolutionWriter":{add_solution_input_vector:[0,2,1,""],add_solution_trajectory:[0,2,1,""],write_to_file:[0,2,1,""]},"commonroad.common.solution_writer.CostFunction":{JB1:[0,4,1,""],SA1:[0,4,1,""],SM1:[0,4,1,""],SM2:[0,4,1,""],SM3:[0,4,1,""],WX1:[0,4,1,""]},"commonroad.common.solution_writer.VehicleModel":{KS:[0,4,1,""],MB:[0,4,1,""],PM:[0,4,1,""],ST:[0,4,1,""]},"commonroad.common.solution_writer.VehicleType":{BMW_320i:[0,4,1,""],FORD_ESCORT:[0,4,1,""],VW_VANAGON:[0,4,1,""]},"commonroad.common.util":{AngleInterval:[0,1,1,""],Interval:[0,1,1,""]},"commonroad.geometry":{shape:[1,0,0,"-"]},"commonroad.geometry.shape":{Circle:[1,1,1,""],Polygon:[1,1,1,""],Rectangle:[1,1,1,""],Shape:[1,1,1,""],ShapeGroup:[1,1,1,""]},"commonroad.geometry.shape.Circle":{center:[1,4,1,""],contains_point:[1,2,1,""],radius:[1,4,1,""],rotate_translate_local:[1,2,1,""],translate_rotate:[1,2,1,""]},"commonroad.geometry.shape.Polygon":{center:[1,4,1,""],contains_point:[1,2,1,""],rotate_translate_local:[1,2,1,""],translate_rotate:[1,2,1,""],vertices:[1,4,1,""]},"commonroad.geometry.shape.Rectangle":{center:[1,4,1,""],contains_point:[1,2,1,""],length:[1,4,1,""],orientation:[1,4,1,""],rotate_translate_local:[1,2,1,""],translate_rotate:[1,2,1,""],vertices:[1,4,1,""],width:[1,4,1,""]},"commonroad.geometry.shape.Shape":{rotate_translate_local:[1,2,1,""],translate_rotate:[1,2,1,""]},"commonroad.geometry.shape.ShapeGroup":{contains_point:[1,2,1,""],rotate_translate_local:[1,2,1,""],shapes:[1,4,1,""],translate_rotate:[1,2,1,""]},"commonroad.planning":{goal:[3,0,0,"-"],planning_problem:[3,0,0,"-"]},"commonroad.planning.goal":{GoalRegion:[3,1,1,""]},"commonroad.planning.goal.GoalRegion":{is_reached:[3,2,1,""],lanelets_of_goal_position:[3,4,1,""],state_list:[3,4,1,""],translate_rotate:[3,2,1,""]},"commonroad.planning.planning_problem":{PlanningProblem:[3,1,1,""],PlanningProblemSet:[3,1,1,""]},"commonroad.planning.planning_problem.PlanningProblem":{goal:[3,4,1,""],goal_reached:[3,2,1,""],initial_state:[3,4,1,""],planning_problem_id:[3,4,1,""],translate_rotate:[3,2,1,""]},"commonroad.planning.planning_problem.PlanningProblemSet":{add_planning_problem:[3,2,1,""],find_planning_problem_by_id:[3,2,1,""],planning_problem_dict:[3,4,1,""],translate_rotate:[3,2,1,""]},"commonroad.prediction":{prediction:[4,0,0,"-"]},"commonroad.prediction.prediction":{Occupancy:[4,1,1,""],Prediction:[4,1,1,""],SetBasedPrediction:[4,1,1,""],TrajectoryPrediction:[4,1,1,""]},"commonroad.prediction.prediction.Occupancy":{shape:[4,4,1,""],time_step:[4,4,1,""],translate_rotate:[4,2,1,""]},"commonroad.prediction.prediction.Prediction":{initial_time_step:[4,4,1,""],occupancy_at_time_step:[4,2,1,""],occupancy_set:[4,4,1,""]},"commonroad.prediction.prediction.SetBasedPrediction":{initial_time_step:[4,4,1,""],occupancy_at_time_step:[4,2,1,""],occupancy_set:[4,4,1,""],translate_rotate:[4,2,1,""]},"commonroad.prediction.prediction.TrajectoryPrediction":{initial_time_step:[4,4,1,""],occupancy_at_time_step:[4,2,1,""],occupancy_set:[4,4,1,""],shape:[4,4,1,""],trajectory:[4,4,1,""],translate_rotate:[4,2,1,""]},"commonroad.scenario":{lanelet:[5,0,0,"-"],obstacle:[5,0,0,"-"],scenario:[5,0,0,"-"],trajectory:[5,0,0,"-"]},"commonroad.scenario.lanelet":{Lanelet:[5,1,1,""],LaneletNetwork:[5,1,1,""],LineMarking:[5,1,1,""]},"commonroad.scenario.lanelet.Lanelet":{adj_left:[5,4,1,""],adj_left_same_direction:[5,4,1,""],adj_right:[5,4,1,""],adj_right_same_direction:[5,4,1,""],all_lanelets_by_merging_successors_from_lanelet:[5,5,1,""],center_vertices:[5,4,1,""],contains_points:[5,2,1,""],convert_to_polygon:[5,2,1,""],distance:[5,4,1,""],get_obstacles:[5,2,1,""],interpolate_position:[5,2,1,""],lanelet_id:[5,4,1,""],left_vertices:[5,4,1,""],line_marking_left_vertices:[5,4,1,""],line_marking_right_vertices:[5,4,1,""],merge_lanelets:[5,5,1,""],predecessor:[5,4,1,""],right_vertices:[5,4,1,""],speed_limit:[5,4,1,""],successor:[5,4,1,""],translate_rotate:[5,2,1,""]},"commonroad.scenario.lanelet.LaneletNetwork":{add_lanelet:[5,2,1,""],add_lanelets_from_network:[5,2,1,""],create_from_lanelet_list:[5,5,1,""],create_from_lanelet_network:[5,5,1,""],filter_obstacles_in_network:[5,2,1,""],find_lanelet_by_id:[5,2,1,""],find_lanelet_by_position:[5,2,1,""],lanelets:[5,4,1,""],lanelets_in_proximity:[5,2,1,""],map_obstacles_to_lanelets:[5,2,1,""],translate_rotate:[5,2,1,""]},"commonroad.scenario.lanelet.LineMarking":{DASHED:[5,4,1,""],SOLID:[5,4,1,""]},"commonroad.scenario.obstacle":{DynamicObstacle:[5,1,1,""],Obstacle:[5,1,1,""],ObstacleRole:[5,1,1,""],ObstacleType:[5,1,1,""],StaticObstacle:[5,1,1,""]},"commonroad.scenario.obstacle.DynamicObstacle":{initial_state:[5,4,1,""],obstacle_id:[5,4,1,""],obstacle_role:[5,4,1,""],obstacle_shape:[5,4,1,""],obstacle_type:[5,4,1,""],occupancy_at_time:[5,2,1,""],prediction:[5,4,1,""],translate_rotate:[5,2,1,""]},"commonroad.scenario.obstacle.Obstacle":{initial_state:[5,4,1,""],obstacle_id:[5,4,1,""],obstacle_role:[5,4,1,""],obstacle_shape:[5,4,1,""],obstacle_type:[5,4,1,""]},"commonroad.scenario.obstacle.ObstacleRole":{DYNAMIC:[5,4,1,""],STATIC:[5,4,1,""]},"commonroad.scenario.obstacle.ObstacleType":{BICYCLE:[5,4,1,""],BUS:[5,4,1,""],CAR:[5,4,1,""],CONSTRUCTION_ZONE:[5,4,1,""],PARKED_VEHICLE:[5,4,1,""],PEDESTRIAN:[5,4,1,""],PRIORITY_VEHICLE:[5,4,1,""],ROAD_BOUNDARY:[5,4,1,""],TRAIN:[5,4,1,""],TRUCK:[5,4,1,""],UNKNOWN:[5,4,1,""]},"commonroad.scenario.obstacle.StaticObstacle":{initial_state:[5,4,1,""],obstacle_id:[5,4,1,""],obstacle_role:[5,4,1,""],obstacle_shape:[5,4,1,""],obstacle_type:[5,4,1,""],occupancy_at_time:[5,2,1,""],translate_rotate:[5,2,1,""]},"commonroad.scenario.scenario":{Scenario:[5,1,1,""]},"commonroad.scenario.scenario.Scenario":{add_objects:[5,2,1,""],benchmark_id:[5,4,1,""],dt:[5,4,1,""],dynamic_obstacles:[5,4,1,""],generate_object_id:[5,2,1,""],lanelet_network:[5,4,1,""],obstacle_by_id:[5,2,1,""],obstacles:[5,4,1,""],obstacles_by_position_intervals:[5,2,1,""],obstacles_by_role_and_type:[5,2,1,""],occupancies_at_time_step:[5,2,1,""],remove_obstacle:[5,2,1,""],static_obstacles:[5,4,1,""],translate_rotate:[5,2,1,""]},"commonroad.scenario.trajectory":{State:[5,1,1,""],Trajectory:[5,1,1,""]},"commonroad.scenario.trajectory.State":{attributes:[5,4,1,""],translate_rotate:[5,2,1,""]},"commonroad.scenario.trajectory.Trajectory":{final_state:[5,4,1,""],initial_time_step:[5,4,1,""],state_at_time_step:[5,2,1,""],state_list:[5,4,1,""],translate_rotate:[5,2,1,""]},"commonroad.visualization":{draw_dispatch_cr:[6,0,0,"-"],plot_helper:[6,0,0,"-"]},"commonroad.visualization.draw_dispatch_cr":{draw_object:[6,6,1,""]},"commonroad.visualization.plot_helper":{redraw_obstacles:[6,6,1,""],set_non_blocking:[6,6,1,""]},"geometry.transform":{from_homogeneous_coordinates:[1,6,1,""],rotate_translate:[1,6,1,""],rotation_translation_matrix:[1,6,1,""],to_homogeneous_coordinates:[1,6,1,""],translate_rotate:[1,6,1,""],translation_rotation_matrix:[1,6,1,""]},geometry:{transform:[1,0,0,"-"]}},objnames:{"0":["py","module","Python module"],"1":["py","class","Python class"],"2":["py","method","Python method"],"3":["py","staticmethod","Python static method"],"4":["py","attribute","Python attribute"],"5":["py","classmethod","Python class method"],"6":["py","function","Python function"]},objtypes:{"0":"py:module","1":"py:class","2":"py:method","3":"py:staticmethod","4":"py:attribute","5":"py:classmethod","6":"py:function"},terms:{"0066cc":[6,10],"1_1_t":8,"1d7eea":10,"2018b":7,"2_1_t":10,"2pi":[0,1],"3399ff":6,"abstract":1,"case":10,"class":8,"default":[6,8,10],"enum":5,"final":5,"float":[1,3,4,5,6],"function":[0,1,5,6,7,8,10],"import":[5,8,10],"int":[0,1,3,4,5,6],"new":[1,8],"return":[0,1,3,4,5,6,8],"static":[0,5,8],"throw":0,"true":[0,1,3,5,10],"while":[8,10],Axes:6,BUS:5,For:[0,8,10],Ids:3,The:[0,1,4,5,7,8,10],These:10,Useful:8,With:7,abc:5,about:[0,6,8],abov:[0,7,10],acceler:[0,5],accord:[5,8,10],achiev:10,activ:10,actual:6,add:[0,3,5,8],add_lanelet:5,add_lanelets_from_network:5,add_object:[5,8],add_planning_problem:3,add_solution_input_vector:[0,8],add_solution_trajectori:[0,8],added:[3,4,5],addit:5,addition:[8,10],adj_left:5,adj_left_same_direct:5,adj_right:5,adj_right_same_direct:5,adjac:5,adjacent_left:5,adjacent_left_same_direct:5,adjacent_right:5,adjacent_right_same_direct:5,affili:[0,8],after:10,algorithm:7,all:[0,1,3,4,5,6,8,10],all_lanelets_by_merging_successors_from_lanelet:5,allow:[0,5,6],along:[4,5,8],alreadi:[0,6],alwai:[0,8],anaconda:7,angl:[0,1,3,4,5,8],angleinterv:[5,8],angular:5,ani:[1,3,5],anoth:8,antialias:10,api:[6,7,10],appear:10,append:8,appli:10,arbitrari:8,arbitrarili:8,area:[4,6,8,10],arg:0,argument:10,around:[1,3,5],arrai:[1,5,8],ask_user_input:0,assign:[5,6],assum:0,attribut:[5,8],author:[0,8],automat:7,avail:7,axes:[6,10],axi:[6,10],back:1,backend:10,base:4,been:5,behavior:[4,8],behaviour:4,belong:10,benchmark:[0,5,7,8],benchmark_id:5,bicycl:5,black:10,block:6,bmw_320i:[0,8],bool:[0,1,3,5],bound:[4,8],boundari:[1,5,8],built:[1,8],bus:5,c7c7c7:10,calcul:8,call:[6,10],call_stack:6,callabl:6,can:[1,4,5,7,8,10],candid:5,cannot:3,canva:10,car:[5,8],center:[1,5,8],center_bound_color:10,center_vertic:5,certain:4,chang:[6,10],check:[0,1,3,5,8],check_validity_of_commonroad_fil:0,circl:[5,6,10],circular:1,classic:10,classmethod:5,clip:10,clockwis:[1,3,4,5,8],code:10,coincid:8,collect:[1,8,10],color:6,combin:10,command:[7,10],common:[2,5,8,10],commonroad:[0,1,3,4,5,6,7,8,10],commonroad_str:0,commonroadfileread:[8,10],commonroadfilewrit:8,commonroadsolutionwrit:8,compar:6,compat:7,complet:[5,6,10],complex:10,compos:[0,5,7],compris:5,comput:[1,4,5],condit:8,connect:[5,8],consid:5,consider:10,consist:[5,8],consol:7,constraint:7,construct:8,construction_zon:5,contain:[0,1,3,5,6,8],contains_point:[1,5],content:7,control:8,convert:[1,5],convert_to_polygon:5,coordin:[1,5],copi:5,correct:[0,8],correspond:[6,8,10],cost:[0,7],cost_funct:[0,8],costfunct:8,could:5,counter:[1,3,4,5,8],counterclockwis:1,creat:[1,5,6,8],create_from_lanelet_list:5,create_from_lanelet_network:5,crop:10,csw:8,current:[1,5,6],customiz:10,dash:5,data:8,date:9,dddddd:10,decimal_precis:0,deep:5,defin:[1,3,4,5,6,8,10],definit:8,deg:8,delet:6,delta_y_f:5,delta_y_r:5,depend:[6,7,10],describ:[1,4,5,8],descript:8,design:10,desir:5,detail:[0,6,8],determin:5,dict:[3,5,6,10],dictionari:5,differ:[5,8],differenti:6,dimension:8,direct:[1,3,4,5],discret:[4,5,8],displac:5,distanc:5,distribut:[4,7],document:[0,6,8],domain:4,done:10,draw:[6,8,10],draw_border_vertic:10,draw_bounding_box:10,draw_center_bound:10,draw_dispatch_cr:[8,10],draw_func:6,draw_icon:10,draw_left_bound:10,draw_linewidth:10,draw_object:[6,8,10],draw_occup:10,draw_param:[6,8],draw_paramet:8,draw_right_bound:10,draw_shap:10,draw_start_and_direct:10,drawn:[6,10],due:5,dure:5,dynam:[0,4,5,6,10],dynamic_obstacl:[5,10],dynamicobstacl:8,each:[0,1,5,7,8],edgecolor:[6,10],effect:10,ego:[0,3,8],either:[4,5,8,10],element:[5,10],els:0,email:7,empti:5,enabl:6,enclos:5,end:0,enough:0,ensur:[6,10],entiti:5,entri:0,enumer:0,environ:8,equal:[8,10],error:[0,3,5],even:[5,10],everi:[6,8,10],exact:[3,5,8],exactli:8,exampl:[5,6,8],exist:[0,5],express:10,extract:10,face_color:10,facecolor:[6,10],fals:[0,1,3,5,10],fast:[6,10],fig:10,figsiz:[8,10],figur:[6,8,10],figure_handl:[6,10],file:[5,7],file_io:0,file_path:8,file_read:[0,8,10],file_writ:[0,8],filenam:[0,8,10],fill_lanelet:10,filter:5,filter_obstacles_in_network:5,final_st:5,find:[3,5,8],find_lanelet_by_id:[5,8],find_lanelet_by_posit:5,find_planning_problem_by_id:3,first:[1,3,5,8],fix:5,follow:[5,8],ford_escort:0,form:[5,8],formal:0,format:0,forward:8,found:[3,8],frame:10,framework:[7,10],from:[0,5,6,7,8,10],from_homogeneous_coordin:1,front:5,fulfil:3,full:6,further:10,furthermor:[5,7,10],futur:[4,5],gca:[8,10],gener:[0,5,6],generate_object_id:5,geometr:1,geometri:[2,5],get:[5,9],get_gca:10,get_obstacl:5,getcwd:[8,10],given:[0,1,3,4,5,6,8,10],global:5,goal:[0,7,8],goal_reach:[3,8],goal_region:[3,8,10],goal_state_1:8,goal_state_2:8,goalregion:[6,8],graphic:10,ground:5,group:1,hand:10,handl:[6,10],has:[3,5,8],have:[5,8,10],height:5,help:10,helper:10,here:7,hierarchi:10,high:10,hire:[],hold:5,homogen:1,howev:5,http:[7,8],i06:7,ids:[3,5],impact:10,implement:[6,7],improv:10,inch_in_cm:10,includ:[0,5,7,8,10],index:[3,7,8],indic:5,inf:5,inform:[0,5,8],init:10,initi:[0,3,4,5,8,10],initial_st:[3,5,10],initial_time_step:[4,5],input:[0,5,8],input_vector:0,insid:[3,8],instanc:[5,8,10],instruct:6,integ:5,interact:[6,10],interior:1,intern:6,interpol:5,interpolate_posit:5,intersect:[1,5,8],interv:[4,5,8],interval_i:5,interval_x:5,introduc:8,invalid:5,is_reach:[3,8],its:[1,4,5,8],itself:10,jb1:0,join:8,jun:9,kei:[3,5],kind:[5,6],known:4,kwarg:5,lane:8,lanelet1:5,lanelet2:5,lanelet:[0,3,6,10],lanelet_id:5,lanelet_network:[5,10],laneletnetwork:[0,6,8],lanelets_in_proxim:[5,8],lanelets_of_goal_posit:3,lankershim:8,last:8,later:[1,5],latter:10,least:3,left:[5,8],left_bound_color:10,left_front_wheel_angular_spe:5,left_rear_wheel_angular_spe:5,left_vertic:5,length:[1,5],level:10,like:[8,10],limit:[6,10],line:5,line_marking_left_vertic:5,line_marking_right_vertic:5,linux:7,list:[0,1,3,4,5,6,7,8,10],locat:5,longitudin:[1,5],loop:10,lowest:10,maco:7,mai:[],main:[6,7,8],maintain:10,manual:[6,7,8],map:5,map_obstacles_to_lanelet:5,maplotlib:10,mark:5,mass:5,match:5,matplotlib:[6,7,8],matrix:1,matter:10,max_length:5,maxmim:5,mean:8,measur:5,merg:5,merge_lanelet:5,messag:5,method:[5,7,8],minim:10,model:[0,1,4,5,7],modifi:[8,10],modul:[7,8],more:[8,10],moreov:10,most:[8,10],motion:[0,5,7],movement:[4,5],mpl:10,name:0,ndarrai:[0,1,3,4,5],necessari:[5,6,8],need:10,neighbor:5,nest:[6,10],network:[0,10],networkx:7,nevertheless:10,ngsim:[8,10],non:6,none:[0,3,4,5,6],note:[0,8],notic:10,number:[5,10],numpi:[5,7,8],obj:6,object:[0,4,5,6,8,10],object_id:6,obstacl:[0,1,4,6,10],obstacle_by_id:5,obstacle_id:[5,6],obstacle_rol:5,obstacle_shap:5,obstacle_typ:5,obstaclerol:8,obstacles_by_position_interv:[5,8],obstacles_by_role_and_typ:5,obstacletyp:8,obtain:5,occup:[5,6,8,10],occupancies_at_time_step:[5,8],occupancy_at_tim:[5,8],occupancy_at_time_step:4,occupancy_set:4,occupi:[1,4,8],offset:1,often:[5,10],omit:[1,10],one:[3,8],onli:[0,5,7,8,10],opac:10,open:[0,8,10],open_lanelet_network:0,option:[0,5,6,8,10],order:[0,1,8,10],org:7,orient:[1,5,8],origin:[1,3,5,10],other:[0,5,8,10],otherwis:[1,4,5],ouput_dir:0,output:0,output_dir:[0,8],over:[4,5,6,8,10],overview:6,overwrit:0,overwrite_existing_fil:0,overwriteexistingfil:8,overwritten:0,packag:7,page:7,param:[],paramet:[0,1,3,4,5,6,8,10],park:8,parked_vehicl:5,particip:5,patch:6,path:8,pdf:[],pedestrian:[1,5,8],per:8,perform:10,perpendicular:5,pip:7,pitch:5,pitch_angl:5,pitch_rat:5,plan:[0,2,6,7,10],plane:5,planning_problem:[3,10],planning_problem_dict:3,planning_problem_id:[0,3,8],planning_problem_list:3,planning_problem_set:[0,8,10],planningproblem:[6,8],planningproblemset:[0,6,8],pleas:[6,8],plot:[6,8],plot_help:10,plot_limit:6,plt:[8,10],pm_input_list:8,pm_state_list:8,png:[],point:[1,5,8,10],point_list:5,pointmass:0,polygon:[5,8,10],polylin:[5,8],posit:[3,4,5,8,10],position_interv:5,position_z:5,position_z_front:5,position_z_rear:5,possibl:[4,5,10],predecessor:5,predict:[2,5],prefer:10,prepar:8,primit:1,print:5,priority_vehicl:5,probabl:4,problem:[0,7,10],project:7,properti:5,provid:[4,5,7,8,10],px1:5,px2:5,py1:5,py2:5,pypi:7,pyplot:[8,10],python:7,qt5agg:10,queri:5,rad:1,radian:[1,3,4,5],radiu:[1,5],rais:[3,5],rang:[8,10],rate:[5,10],reach:[3,8],reachabl:5,read:[0,7],real:5,rear:5,recommend:7,recreat:6,rectangl:[8,10],rectangular:1,redraw_dynamic_obstacl:10,redraw_obstacl:[6,10],redrawn:10,refer:[6,10],region:[0,1,8],relat:5,releas:[4,7,9],remov:5,remove_obstacl:[5,8],repres:[1,4,5,8],represent:[0,5],requir:[3,7,10],restrict:10,right:[5,8],right_bound_color:10,right_front_wheel_angular_spe:5,right_rear_wheel_angular_spe:5,right_vertic:5,road:[0,8],road_boundari:5,role:5,roll:5,roll_angl:5,roll_angle_front:5,roll_angle_rear:5,roll_rat:5,roll_rate_front:5,roll_rate_rear:5,rotat:[1,3,4,5,8],rotate_transl:1,rotate_translate_loc:1,rotation_translation_matrix:1,rout:5,run:[7,10],runtim:[5,10],sa1:[0,8],same:[6,10],satisfi:5,save:6,scenario:[0,2,6,7,10],scenario_id:[0,8],scenario_object:5,schema:[0,8],search:[3,7,10],second:5,see:[5,6,8],segment:8,select:10,self:3,sensor:5,sequenc:4,set:[0,1,3,4,5,6,10],set_aspect:8,set_non_block:[6,10],set_xlim:10,setbasedpredict:8,sever:5,shape:[4,5,6,7,8,10],shape_paramet:10,should:[0,6,10],show:[8,10],show_label:10,shown:10,simpl:10,simplifi:10,sinc:10,singl:[5,10],size:5,skip:0,slip:5,slip_angl:5,slot:5,slow:10,sm1:0,sm2:0,sm3:0,small:[],solid:5,solut:7,solution_writ:[0,8],solv:[0,7,8],some:8,sourc:[0,1,3,4,5,6,8,10],specif:[4,5,10],specifi:[0,1,5,6,10],speed:5,speed_limit:5,sprung:5,stack:6,start:[0,5,9,10],state:[0,3,4,6,8],state_at_time_step:5,state_list:[3,5],static_obstacl:[5,6,10],staticobstacl:8,steer:5,steering_angl:5,steering_angle_spe:0,step:[4,5,8],step_siz:[0,8],store:[5,8],str:[0,5,6],string:[0,6],structur:[6,10],style:[6,10],submit:8,subset:5,successfulli:5,successor:[5,8],suffici:10,superclass:5,support:4,surround:8,system:5,tag:[0,8],taken:0,term:0,test:[3,7],test_scenario:8,than:10,thei:10,them:[5,7,10],therefor:[1,10],therein:10,thi:[5,6,8,10],those:[7,8,10],though:5,three:[0,4],through:5,time:[0,4,5,6,8],time_begin:[8,10],time_end:10,time_step:[0,4,5,8],tkagg:10,to_homogeneous_coordin:1,too:10,traffic:5,train:5,trajectori:[0,3,4,6],trajectory_pm:8,trajectory_step:10,trajectorypredict:8,transform:5,translat:[1,3,4,5,8],translate_rot:[1,3,4,5,8],translation_rotation_matrix:1,tree:0,truck:5,tum:[0,7,8],tupl:[0,3,5,6],tutori:7,two:[5,8],type:[0,1,3,4,5,6,8,10],typic:10,unambigu:10,uncertain:5,union:[1,3,4,5,6],uniqu:[4,5,8],unknown:[4,5],updat:[6,10],upload:[0,7,8],us101:10,usa_lank:8,usa_us101:10,usag:7,use:10,used:[1,5,6,7,8,10],useful:8,user:[7,10],using:[4,6,7,10],usual:6,util:[5,8],valid:0,valu:[0,3,5],valueerror:5,variabl:5,vector:[0,1,3,4,5,8],vehicl:[0,1,3,5,7,8],vehicle_model:[0,8],vehicle_typ:[0,8],vehiclemodel:8,vehicletyp:8,veloc:[5,8],velocity_i:[5,8],velocity_y_front:5,velocity_y_rear:5,velocity_z:5,velocity_z_front:5,velocity_z_rear:5,version:4,vertic:[1,5,8],via:7,visual:[2,7,8,9],vw_vanagon:0,wai:8,warn:5,websit:7,well:8,wheel:5,when:[7,10],where:10,whether:0,which:[0,1,5,6,8,10],whole:10,width:1,within:[4,5,8],without:0,work:8,would:10,write:[0,7,10],write_scenario_to_fil:0,write_scenario_to_file_io:0,write_to_fil:[0,8],write_to_file_io:0,written:[7,8],wx1:0,x_0:1,x_1:1,x_acceler:[0,8],x_c:5,x_l:5,x_max:[6,10],x_min:[6,10],x_off:[1,3,4,5],x_r:5,xml:[0,4,5,7,8,10],xml_string:0,xsd:0,y_0:1,y_1:1,y_acceler:[0,8],y_c:5,y_l:5,y_max:[6,10],y_min:[6,10],y_off:[1,3,4,5],y_r:5,yaw:5,yaw_rat:5,you:10,zero:1,zone:8,zorder:10},titles:["Module Common","Module Geometry","CommonRoad_io API","Module Planning","Module Prediction","Module Scenario","Module Visualization","CommonRoad_io Documentation","Getting Started","User Manual","Visualization Manual"],titleterms:{"class":[0,1,3,4,5],angleinterv:0,api:2,applic:10,base:8,circl:1,common:0,commonroad_io:[2,7],commonroadfileread:0,commonroadfilewrit:0,commonroadsolutionwrit:0,contact:7,costfunct:0,custom:10,document:7,draw_dispatch_cr:6,draw_param:10,dynam:8,dynamicobstacl:5,exampl:10,file:[0,8],geometri:1,get:[7,8],goal:3,goalregion:3,indic:7,inform:7,instal:7,interv:0,lanelet:[5,8],laneletnetwork:5,linemark:5,manual:[9,10],matplotlib:10,modul:[0,1,2,3,4,5,6],network:[5,8],obstacl:[5,8],obstaclerol:5,obstacletyp:5,occup:4,overwriteexistingfil:0,pass:10,plan:[3,8],planningproblem:3,planningproblemset:3,plot:10,plot_help:6,plot_limit:10,polygon:1,predict:[4,8],problem:[3,8],read:8,reader:0,real:10,rectangl:1,region:3,road:5,scenario:[5,8],set:8,setbasedpredict:4,shape:1,shapegroup:1,solut:[0,8],speed:10,start:[7,8],state:5,staticobstacl:5,tabl:7,time:10,trajectori:[5,8],trajectorypredict:4,transform:1,user:9,util:0,vehiclemodel:0,vehicletyp:0,visual:[6,10],write:8,writer:[0,8]}})