#!/usr/bin/python


#header files for SQL data
import rospy
import sys
from std_msgs.msg import Bool
from wms_data.msg import WMSData
from wms_data.msg import WMSTasks
from wms_data.msg import Feedback
import uuid 
import requests
import json
import logging

logging.basicConfig(filename='/home/nvidia/catkin_ws/src/anscer_agv/wms_data/logs/wms.logs', encoding='utf-8', level=logging.DEBUG)

url = ""
updatecount=0
failUpdatecount=0
global lastUID
lastUID=""
global lastAction
lastAction=-1
global idRecord
global actionRecord
global acr_shelfRecord
idRecord = ""
actionRecord = 0
acr_shelfRecord = 0

global updateFlag
updateFlag=True

WMS_pick_DataPub=rospy.Publisher('/wms_pick_tasks',WMSTasks,queue_size=1)
WMS_drop_DataPub=rospy.Publisher('/wms_drop_tasks',WMSTasks,queue_size=1)
WMS_statusPub=rospy.Publisher('/wms_status',Bool,queue_size=1)

####
##@brief API call for reading picking tasks
##

def readPickingData(url):
    wmsPickTaskArray=WMSTasks()
    
    
    unique_id=""
    date=""
    priority=0
    rack=0
    shelf=0
    bin=0
    action=1
    status=0
    failStatus=0
    ean_code=""
    putaway_code=""
    acr_shelf = 0
    whole_bin_no=""
    core=""
    level=""
    year=""
    date_data=""
    serial_no=""
    
  
# structured XML
    payload = """<?xml version="1.0" encoding="utf-8"?>
        <soap:Envelope xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:xsd="http://www.w3.org/2001/XMLSchema" xmlns:soap="http://schemas.xmlsoap.org/soap/envelope/">
        <soap:Body>
        <getWMSPickDataNew xmlns="http://tempuri.org/" />
        </soap:Body>
        </soap:Envelope>"""

# headers
    headers = {
                'Content-Type': 'text/xml; charset=utf-8'
                }
# POST request
    try:
        print("requsting")
        response = requests.request("POST", url, headers=headers, data=payload,timeout=10)
        #print(response)
        data=""
        data=response.content
        #print(data)
        delimiter1="getWMSPickDataNewResult"
        delimiter2="</getWMSPickDataNewResult"
        
        pickData=convertToJson(data,delimiter1,delimiter2)
        #print(pickData)
        
        for i in pickData:
            wmsPickTask=WMSData()
            ######
            #### reading data from json file
            #####
            unique_id=str(i['Inst_Kid'])
            date     =str(i['Inst_Date'])
            priority =int(i['Inst_Priority'])
            rack     =int(i['Inst_RackNo'])
            shelf    =int(i['Inst_ShelfNo'])
            bin      =int(i['Inst_BinNo'])
            action   =1
            status   =int(i['Inst_IsCompleted'])
            ean_code =str(i['Inst_EANCode'])
            putaway_code=str(i['Inst_PutawayBarcode'])
            if(i['Inst_BotShelfNo']=="None" or i['Inst_BotShelfNo']==""):
                acr_shelf=0
            else:
                acr_shelf   =int(i['Inst_BotShelfNo'])
            
            whole_bin_no=str(i['Inst_WholeBinNo'])
            core        ="None"
            level       ="None"
            year        ="None"
            date_data   ="None"
            serial_no   ="None"
            
            ####
            ## storing the read data in to wmsdata variable
            ####
            wmsPickTask.unique_id =unique_id
            wmsPickTask.date = date
            wmsPickTask.priority = priority
            wmsPickTask.rack = rack
            wmsPickTask.shelf = 1 + (shelf % 10)
            wmsPickTask.bin = bin
            wmsPickTask.action = action
            wmsPickTask.ean_code=ean_code
            wmsPickTask.putaway_code=putaway_code
            wmsPickTask.whole_bin_no=whole_bin_no
            wmsPickTask.core=core
            wmsPickTask.level=level
            wmsPickTask.year=year
            wmsPickTask.date_data=date_data
            wmsPickTask.serial_no=serial_no
            wmsPickTask.status = status
            
            
    
            wmsPickTaskArray.tasks.append(wmsPickTask)
            #print(wmsPickTaskArray)
    # print (wmsPickTaskArray)
        WMS_pick_DataPub.publish(wmsPickTaskArray)
    except:
        rospy.logerr('/* pick task updating error */')
        pass
        
###
##@brief API call for reading droping tasks
##        
        
def readDropingData(url):
    
    
    wmsDropTaskArray=WMSTasks()
  #  url = "http://192.168.1.4/AraplWebServiceACR/Services.asmx"
  
# structured XML
    payload = """<?xml version="1.0" encoding="utf-8"?>
        <soap:Envelope xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:xsd="http://www.w3.org/2001/XMLSchema" xmlns:soap="http://schemas.xmlsoap.org/soap/envelope/">
        <soap:Body>
        <getWMSDropDataNew xmlns="http://tempuri.org/" />
        </soap:Body>
        </soap:Envelope>"""
# headers
    headers = {
        'Content-Type': 'text/xml; charset=utf-8'
        }
# POST request
    try:
        response = requests.request("POST", url, headers=headers, data=payload,timeout=10)

        data=""
        data=response.content
        delimiter1="getWMSDropDataNewResult"
        delimiter2="</getWMSDropDataNewResult"
        
        dropData=convertToJson(data,delimiter1,delimiter2)
    #  print(dropData)
        for i in dropData:
            wmsDropTask=WMSData()
            ######
            #### reading data from json file
            #####
            unique_id=str(i['Inst_Kid'])
            date     =str(i['Inst_Date'])
            priority =int(i['Inst_ToPriority'])
            rack     =int(i['Inst_ToRackNo'])
            shelf    =int(i['Inst_ToShelfNo'])
            bin      =int(i['Inst_ToBinNo'])
            action   =1
            status   =int(i['Inst_ToIsCompleted'])
            ean_code =str(i['Inst_EANCode'])
            putaway_code=str(i['Inst_PutawayBarcode'])
            if(i['Inst_BotShelfNo']=="None" or i['Inst_BotShelfNo']==""):
                acr_shelf=0
            else:
                acr_shelf   =int(i['Inst_BotShelfNo'])
            whole_bin_no=str(i['Inst_ToWholeBinNo'])
            core        ="None"
            level       ="None"
            year        ="None"
            date_data   ="None"
            serial_no   ="None"
            
            ####
            ## storing the read data in to wmsdata variable
            ####
            
            wmsDropTask.unique_id =unique_id
            wmsDropTask.date = date
            wmsDropTask.priority = priority
            wmsDropTask.rack = rack
            wmsDropTask.shelf = 1 + (shelf % 10)
            wmsDropTask.bin = bin
            wmsDropTask.action = action
            wmsDropTask.ean_code=ean_code
            wmsDropTask.putaway_code=putaway_code
            wmsDropTask.acr_shelf=acr_shelf
            wmsDropTask.whole_bin_no=whole_bin_no
            wmsDropTask.core=core
            wmsDropTask.level=level
            wmsDropTask.year=year
            wmsDropTask.date_data=date_data
            wmsDropTask.serial_no=serial_no
            wmsDropTask.status = status
            wmsDropTaskArray.tasks.append(wmsDropTask)

            #print (wmsTasks)
        WMS_drop_DataPub.publish(wmsDropTaskArray)
            
    except:
        rospy.logerr('/* Drop data updating failed */')
        pass
        
        
###
## @brief API call for updating tasks completion
###       

def updateData(url,Inst_kid,action,botShelf):
    global actionRecord
    global updatecount
    global updateFlag
    global actionRecord
    global acr_shelfRecord
    global idRecord
    actionRecord=action
    idRecord=Inst_kid
    acr_shelfRecord=botShelf
    feedback=""
    feedbackData=False
    uid=Inst_kid
    Action=int(action)
    BotShelf=str(botShelf)
    URL=url
    updateFlag=False
    rospy.loginfo("WMS update request recieved")
    
  #  url = "http://192.168.1.4/AraplWebServiceACR/Services.asmx"
  
# structured XML
    try:
        payload = """<?xml version="1.0" encoding="utf-8"?>
        <soap:Envelope xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:xsd="http://www.w3.org/2001/XMLSchema" xmlns:soap="http://schemas.xmlsoap.org/soap/envelope/">
        <soap:Body>
            <UpdateWMSData xmlns="http://tempuri.org/">
            <Kid>{}</Kid>
            <BotShelf>{}</BotShelf>
            <Action>{}</Action>
            </UpdateWMSData>
        </soap:Body>
        </soap:Envelope>""".format(uid,BotShelf,Action)

    # headers
        headers = {
            'Content-Type': 'text/xml; charset=utf-8'
        }
    # POST request

        response = requests.request("POST", url, headers=headers, data=payload,timeout=10)

        delimiter1="UpdateWMSDataResult"
        delimiter2="</UpdateWMSDataResult>"
        message=""
        message=response.content
        # print(message)
        #print(response.content)  
        feedback= extractFeedback(message,delimiter1,delimiter2)
       # print("wms feedback :",feedback)
       # print(type(feedback))
        rospy.loginfo("feedback -----------------------")
        rospy.loginfo(uid)
        rospy.loginfo(feedback)
        if(feedback == "true"):
            feedbackData=True
            rospy.loginfo("Recived True from WMS after update")
            updateFlag = True
            WMS_statusPub.publish(feedbackData)
        
        else:
            feedbackData=False
          #  updateFlag=False
            rospy.loginfo("recived False from wms after update")
            rospy.loginfo(uid)
    except:
        rospy.logerr('/* Lifter Feedback error */')
        logging.debug('/* lifter Feed back  communication */')
        #feedbackData = False
        pass
    WMS_statusPub.publish(feedbackData)

    logging.debug('unique id  =  ')
    logging.debug(uid)
    logging.debug('Action  =  ')
    logging.debug(Action)
    logging.debug('feedback  =  ')
    logging.debug(feedback)
   # if(feedbackData==False and updatecount<3):
    #    updateData(URL,Inst_kid,action,botShelf)
     #   updatecount=updatecount+1    
    
    
    
#
#callbackfunction definition for lifter feedback
#
def liftFeedbackCallback(feedbackMsg):
    global url
    global   instructionIsCompleted
    global botShelf
    global action
    global failStatus
    global updatecount
    global failUpdatecount
    global lastUID
    global lastAction
    updatecount=0
    failUpdatecount=0
    instructionIsCompleted=0
    action=-1
    botShelf=0
    failStatus=0
    
    Inst_kid=feedbackMsg.unique_id
    print(Inst_kid)
    action=feedbackMsg.action
    print(action)
    botShelf=feedbackMsg.acr_shelf
    print(botShelf)
    instructionIsCompleted=feedbackMsg.status
    print(instructionIsCompleted)
    failStatus=feedbackMsg.failStatus
    # WMSWriteData(Inst_kid,Inst_acrShelf)
    if((instructionIsCompleted==1 and failStatus==0) and (lastUID!=Inst_kid or lastAction!=action)):
        if((action==1 and botShelf!=0) or action==0):
            lastUID=Inst_kid
            lastAction=action
            updateData(url,Inst_kid,action,botShelf)
    elif(instructionIsCompleted==1 and failStatus==1):
      #  lastUID=Inst_kid
        failUpdateData(url,Inst_kid,action)
 
###
##@brief failstatus update
###
def failUpdateData(url,Inst_kid,action):
    global failUpdatecount    
    error=""
    feedback=""
    feedbackData=False 
    uid=Inst_kid
    Action=int(action)
    URL=url
    if (action==1):
        error="unable to scan bin barcode"
    elif(action==0):
        error="unable to scan bin barcode"
  #  url = "http://192.168.1.4/AraplWebServiceACR/Services.asmx"
  
# structured XML
    try:
        payload = """<?xml version="1.0" encoding="utf-8"?>
        <soap:Envelope xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:xsd="http://www.w3.org/2001/XMLSchema" xmlns:soap="http://schemas.xmlsoap.org/soap/envelope/">
        <soap:Body>
        <UpdateBOTErrorLogData xmlns="http://tempuri.org/">
        <Kid>{}</Kid>
        <Remarks>{}</Remarks>
        </UpdateBOTErrorLogData>
        </soap:Body>
        </soap:Envelope>""".format(uid,error)

    # headers
        headers = {
            'Content-Type': 'text/xml; charset=utf-8'
        }
    # POST request

        response = requests.request("POST", url, headers=headers, data=payload,timeout=10)

    #   print(response.content) 
        

        delimiter1="UpdateBOTErrorLogDataResult"
        delimiter2="</UpdateBOTErrorLogDataResult>"
        message=""    
        message=response.content
        # print(message)

        feedback= extractFeedback(message,delimiter1,delimiter2)
        if(feedback == "true"):
            feedbackData=True
            WMS_statusPub.publish(feedbackData)
        else:
            feedbackData=False
    except:
        rospy.logerr('/* Update checking  */')
        pass
    #WMS_statusPub.publish(feedbackData)
    #if(feedbackData==False and failUpdatecount<3):
     #   failUpdateData(URL,Inst_kid,action,botShelf)
      #  failUpdatecount=failUpdatecount+1
     
 
 
 ###
 #for extracting data from feedback
 #
def extractFeedback(message,delimiter1,delimiter2):
     # getting index of substrings
    # print(message)
    idx1 = message.index(delimiter1)
    idx2 = message.index(delimiter2)
    res = ''
    # getting elements in between
    for idx in range(idx1 + len(delimiter1) + 1, idx2):
        res = res + message[idx]
    print("wms feedback : " + res)
    return str(res)
 
        
####
### for converting the data to json
###

def convertToJson(data,delimiter1,delimiter2):
    
#print(data)


    sub1 = delimiter1
    sub2 = delimiter2
 
# getting index of substrings
    idx1 = data.index(sub1)
    idx2 = data.index(sub2)
 
    res = ''
# getting elements in between
    for idx in range(idx1 + len(sub1) + 1, idx2):
        res = res + data[idx]
 
# printing result
#print("The extracted string : " + res)

    jsondata=json.loads(res)

# print(response.content)
#print(jsondata)

    # for i in jsondata:
    #     print(i)
    #     print("\n")
    return jsondata


    
##
#dataPublisher() definition
#

def dataPublisherAndSubscriber():
    global url
    global updateFlag
    url="http://192.168.1.10/wms_webservice/Services.asmx"
    #url="http://localhost:3333/wms_webservice/Services.asmx"
    rospy.Subscriber("/lifterFeedback", Feedback, liftFeedbackCallback)
    rospy.init_node('wms_database_node',anonymous=True)
    rate=rospy.Rate(1)
    while not rospy.is_shutdown():
        if(updateFlag!=True):
            rospy.sleep(1)
            rospy.loginfo("Updating WMS failed retrying")
            updateData(url,idRecord,actionRecord,acr_shelfRecord)
        readPickingData(url)
        readDropingData(url)
        rospy.loginfo("info message")
        rate.sleep()


 
##
# main function definition
# 

if __name__ == "__main__":
   # while not rospy.is_shutdown():
        try:
            print("wmsDatabase node started")
            dataPublisherAndSubscriber()
        except rospy.ROSInterruptException:
            pass
