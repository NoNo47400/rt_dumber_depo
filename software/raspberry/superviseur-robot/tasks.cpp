/*
 * Copyright (C) 2018 dimercur
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "tasks.h"
#include <stdexcept>

// Déclaration des priorités des taches
#define PRIORITY_TSERVER 30
#define PRIORITY_TOPENCOMROBOT 20
#define PRIORITY_TMOVE 20
#define PRIORITY_TSENDTOMON 22
#define PRIORITY_TRECEIVEFROMMON 25
#define PRIORITY_TSTARTROBOT 20
#define PRIORITY_TCAMERA 21
#define PRIORITY_BATTERY 19
#define PRIORITY_START_CAMERA 20
#define PRIORITY_ENVOI_CAMERA 20
#define PRIORITY_SEARCH_ARENA 19
#define PRIORITY_CALCULATE_POSITION 19



/*
 * Some remarks:
 * 1- This program is mostly a template. It shows you how to create tasks, semaphore
 *   message queues, mutex ... and how to use them
 * 
 * 2- semDumber is, as name say, useless. Its goal is only to show you how to use semaphore
 * 
 * 3- Data flow is probably not optimal
 * 
 * 4- Take into account that ComRobot::Write will block your task when serial buffer is full,
 *   time for internal buffer to flush
 * 
 * 5- Same behavior existe for ComMonitor::Write !
 * 
 * 6- When you want to write something in terminal, use cout and terminate with endl and flush
 * 
 * 7- Good luck !
 */

/**
 * @brief Initialisation des structures de l'application (tâches, mutex, 
 * semaphore, etc.)
 */
void Tasks::Init() {
    int status;
    int err;

    /**************************************************************************************/
    /* 	Mutex creation                                                                    */
    /**************************************************************************************/
    if (err = rt_mutex_create(&mutex_monitor, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robot, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robotStarted, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_move, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_CamOpened, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_CamOpenAutorisation, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_Camera, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_SearchingArena, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_UseArena, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_RobotPosition, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_ArenaResult, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    
    cout << "Mutexes created successfully" << endl << flush;

    /**************************************************************************************/
    /* 	Semaphors creation       							  */
    /**************************************************************************************/
    if (err = rt_sem_create(&sem_barrier, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_openComRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_serverOk, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    /*if (err = rt_sem_create(&sem_openCam, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }*/
    
    cout << "Semaphores created successfully" << endl << flush;

    /**************************************************************************************/
    /* Tasks creation                                                                     */
    /**************************************************************************************/
    if (err = rt_task_create(&th_server, "th_server", 0, PRIORITY_TSERVER, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_sendToMon, "th_sendToMon", 0, PRIORITY_TSENDTOMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_receiveFromMon, "th_receiveFromMon", 0, PRIORITY_TRECEIVEFROMMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_openComRobot, "th_openComRobot", 0, PRIORITY_TOPENCOMROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_startRobot, "th_startRobot", 0, PRIORITY_TSTARTROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_move, "th_move", 0, PRIORITY_TMOVE, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_battery_state, "th_battery_state", 0, PRIORITY_BATTERY, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_start_camera, "th_start_camera", 0, PRIORITY_START_CAMERA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_envoi_img, "th_envoi_camera", 0, PRIORITY_ENVOI_CAMERA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_search_my_arena, "th_search_my_arena", 0, PRIORITY_SEARCH_ARENA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_calculate_position, "th_calculate_position", 0, PRIORITY_CALCULATE_POSITION, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    
    cout << "Tasks created successfully" << endl << flush;

    /**************************************************************************************/
    /* Message queues creation                                                            */
    /**************************************************************************************/
    if ((err = rt_queue_create(&q_messageToMon, "q_messageToMon", sizeof (Message*)*50, Q_UNLIMITED, Q_FIFO)) < 0) {
        cerr << "Error msg queue create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Queues created successfully" << endl << flush;

}

/**
 * @brief Démarrage des tâches
 */
void Tasks::Run() {
    rt_task_set_priority(NULL, T_LOPRIO);
    int err;

    if (err = rt_task_start(&th_server, (void(*)(void*)) & Tasks::ServerTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_sendToMon, (void(*)(void*)) & Tasks::SendToMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_receiveFromMon, (void(*)(void*)) & Tasks::ReceiveFromMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_openComRobot, (void(*)(void*)) & Tasks::OpenComRobot, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_startRobot, (void(*)(void*)) & Tasks::StartRobotTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_move, (void(*)(void*)) & Tasks::MoveTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_battery_state, (void(*)(void*)) & Tasks::BatteryState, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_start_camera, (void(*)(void*)) & Tasks::StartCamera, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_envoi_img, (void(*)(void*)) & Tasks::EnvoiImg, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_search_my_arena, (void(*)(void*)) & Tasks::SearchMyArena, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_calculate_position, (void(*)(void*)) & Tasks::CalculatePosition, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Tasks launched" << endl << flush;
}

/**
 * @brief Arrêt des tâches
 */
void Tasks::Stop() {
    monitor.Close();
    robot.Close();
}

/**
 */
void Tasks::Join() {
    cout << "Tasks synchronized" << endl << flush;
    rt_sem_broadcast(&sem_barrier);
    pause();
}

/**
 * @brief Thread handling server communication with the monitor.
 */
void Tasks::ServerTask(void *arg) {
    int status;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are started)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task server starts here                                                        */
    /**************************************************************************************/
    rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
    status = monitor.Open(SERVER_PORT);
    rt_mutex_release(&mutex_monitor);

    cout << "Open server on port " << (SERVER_PORT) << " (" << status << ")" << endl;

    if (status < 0) throw std::runtime_error {
        "Unable to start server on port " + std::to_string(SERVER_PORT)
    };
    monitor.AcceptClient(); // Wait the monitor client
    cout << "Rock'n'Roll baby, client accepted!" << endl << flush;
    rt_sem_broadcast(&sem_serverOk);
}

/**
 * @brief Thread sending data to monitor.
 */
void Tasks::SendToMonTask(void* arg) {
    Message *msg;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task sendToMon starts here                                                     */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);

    while (1) {
        cout << "wait msg to send" << endl << flush;
        msg = ReadInQueue(&q_messageToMon);
        cout << "Send msg to mon: " << msg->ToString() << endl << flush;
        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
        monitor.Write(msg); // The message is deleted with the Write
        rt_mutex_release(&mutex_monitor);
    }
}

/**
 * @brief Thread receiving data from monitor.
 */
void Tasks::ReceiveFromMonTask(void *arg) {
    Message *msgRcv;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task receiveFromMon starts here                                                */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);
    cout << "Received message from monitor activated" << endl << flush;

    while (1) {
        msgRcv = monitor.Read();
        cout << "Rcv <= " << msgRcv->ToString() << endl << flush;

        if (msgRcv->CompareID(MESSAGE_MONITOR_LOST)) {
            delete(msgRcv);
            exit(-1);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_COM_OPEN)) {
            rt_sem_v(&sem_openComRobot);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITHOUT_WD)) {
            rt_sem_v(&sem_startRobot);
        } else if (msgRcv->CompareID(MESSAGE_CAM_OPEN)) {
            rt_mutex_acquire(&mutex_CamOpenAutorisation, TM_INFINITE);
            CamOpenAutorisation = true;
            rt_mutex_release(&mutex_CamOpenAutorisation);
        } else if (msgRcv->CompareID(MESSAGE_CAM_CLOSE)) {
            rt_mutex_acquire(&mutex_CamOpenAutorisation, TM_INFINITE);
            CamOpenAutorisation = false;
            rt_mutex_release(&mutex_CamOpenAutorisation);
        }
        else if (msgRcv->CompareID(MESSAGE_CAM_ASK_ARENA)) {
            rt_mutex_acquire(&mutex_SearchingArena, TM_INFINITE);
            SearchingArena = true;
            rt_mutex_release(&mutex_SearchingArena);
        } else if (msgRcv->CompareID(MESSAGE_CAM_ARENA_CONFIRM)) {
            rt_mutex_acquire(&mutex_SearchingArena, TM_INFINITE);
            ArenaValid = true;
            rt_mutex_release(&mutex_SearchingArena);
        } else if (msgRcv->CompareID(MESSAGE_CAM_ARENA_INFIRM)) {
            rt_mutex_acquire(&mutex_SearchingArena, TM_INFINITE);
            ArenaValid = false;
            rt_mutex_release(&mutex_SearchingArena);
        } else if (msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_START)) {
            rt_mutex_acquire(&mutex_CalculPosition, TM_INFINITE);
            CalculPosition = true;
            rt_mutex_release(&mutex_CalculPosition);
        } else if (msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_STOP)) {
            rt_mutex_acquire(&mutex_CalculPosition, TM_INFINITE);
            CalculPosition = false;
            rt_mutex_release(&mutex_CalculPosition);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_GO_FORWARD) ||
                    msgRcv->CompareID(MESSAGE_ROBOT_GO_BACKWARD) ||
                    msgRcv->CompareID(MESSAGE_ROBOT_GO_LEFT) ||
                    msgRcv->CompareID(MESSAGE_ROBOT_GO_RIGHT) ||
                    msgRcv->CompareID(MESSAGE_ROBOT_STOP)) {
            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            move = msgRcv->GetID();
            rt_mutex_release(&mutex_move);
        }
        delete(msgRcv); // mus be deleted manually, no consumer
    }
}

/**
 * @brief Thread opening communication with the robot.
 */
void Tasks::OpenComRobot(void *arg) {
    int status;
    int err;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task openComRobot starts here                                                  */
    /**************************************************************************************/
    while (1) {
        rt_sem_p(&sem_openComRobot, TM_INFINITE);
        cout << "Open serial com (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        status = robot.Open();
        rt_mutex_release(&mutex_robot);
        cout << status;
        cout << ")" << endl << flush;

        Message * msgSend;
        if (status < 0) {
            msgSend = new Message(MESSAGE_ANSWER_NACK);
        } else {
            msgSend = new Message(MESSAGE_ANSWER_ACK);
        }
        WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon
    }
}

/**
 * @brief Thread starting the communication with the robot.
 */
void Tasks::StartRobotTask(void *arg) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task startRobot starts here                                                    */
    /**************************************************************************************/
    while (1) {

        Message * msgSend;
        rt_sem_p(&sem_startRobot, TM_INFINITE);
        cout << "Start robot without watchdog (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        msgSend = robot.Write(robot.StartWithoutWD());
        rt_mutex_release(&mutex_robot);
        cout << msgSend->GetID();
        cout << ")" << endl;

        cout << "Movement answer: " << msgSend->ToString() << endl << flush;
        WriteInQueue(&q_messageToMon, msgSend);  // msgSend will be deleted by sendToMon

        if (msgSend->GetID() == MESSAGE_ANSWER_ACK) {
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 1;
            rt_mutex_release(&mutex_robotStarted);
        }
    }
}

/**
 * @brief Thread handling control of the robot.
 */
void Tasks::MoveTask(void *arg) {
    int rs;
    int cpMove;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 100000000);

    while (1) {
        rt_task_wait_period(NULL);
        cout << "Periodic movement update";
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1) {
            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            cpMove = move;
            rt_mutex_release(&mutex_move);
            
            cout << " move: " << cpMove;
            
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            robot.Write(new Message((MessageID)cpMove));
            rt_mutex_release(&mutex_robot);
        }
        cout << endl << flush;
    }
}

/**
 * Write a message in a given queue
 * @param queue Queue identifier
 * @param msg Message to be stored
 */
void Tasks::WriteInQueue(RT_QUEUE *queue, Message *msg) {
    int err;
    if ((err = rt_queue_write(queue, (const void *) &msg, sizeof ((const void *) &msg), Q_NORMAL)) < 0) {
        cerr << "Write in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in write in queue"};
    }
}

/**
 * Read a message from a given queue, block if empty
 * @param queue Queue identifier
 * @return Message read
 */
Message *Tasks::ReadInQueue(RT_QUEUE *queue) {
    int err;
    Message *msg;

    if ((err = rt_queue_read(queue, &msg, sizeof ((void*) &msg), TM_INFINITE)) < 0) {
        cout << "Read in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in read in queue"};
    }/** else {
        cout << "@msg :" << msg << endl << flush;
    } /**/

    return msg;
}

/**
 * @brief Pour recup le niveau de batterie
 */
void Tasks::BatteryState(void *arg) {
    int rs;
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    MessageBattery * msg;
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 500000000);
    while (1) {
        rt_task_wait_period(NULL);
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1) {
            cout << "Periodic battery state";
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msg = (MessageBattery*)robot.Write(new Message(MESSAGE_ROBOT_BATTERY_GET));
            rt_mutex_release(&mutex_robot);
            cout << " write battery state: " << msg;
            rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
            monitor.Write(msg);
            rt_mutex_release(&mutex_monitor);
        }
        cout << endl << flush;
    }
}   

void Tasks::StartCamera(void *arg){
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    Message *msg_to_mon;
    // Création d'une caméra
    cam = new Camera(sm,10);
    bool cam_statut_local = false;
    bool cam_open_autorisation = false;
    while (1) {
        //rt_sem_p(&sem_openCam, TM_INFINITE);  // Peut-etre a changer lorsqu'on rajoutera le stop camera car besoin d'un mutex ou d'une autre tache
        rt_mutex_acquire(&mutex_CamOpenAutorisation, TM_INFINITE);
        cam_open_autorisation = CamOpenAutorisation;
        rt_mutex_release(&mutex_CamOpenAutorisation);
        if (cam_open_autorisation) {
            rt_mutex_acquire(&mutex_Camera, TM_INFINITE);
            cam_statut_local = cam->Open();
            rt_mutex_release(&mutex_Camera);
            rt_mutex_acquire(&mutex_CamOpened, TM_INFINITE);
            CamOpened = cam_statut_local;
            rt_mutex_release(&mutex_CamOpened);
            if (cam_statut_local) {
                msg_to_mon = new Message(MESSAGE_ANSWER_ACK);
            }
            else {
                msg_to_mon = new Message(MESSAGE_ANSWER_NACK);
            }
        }
        else {
            rt_mutex_acquire(&mutex_CamOpened, TM_INFINITE);
            CamOpened = false;
            rt_mutex_release(&mutex_CamOpened);
            rt_mutex_acquire(&mutex_Camera, TM_INFINITE);
            cam->Close();
            rt_mutex_release(&mutex_Camera);
            msg_to_mon = new Message(MESSAGE_ANSWER_ACK);
        }
        // A REMETTRE MAIS PROBLM DE SEGMENT FAULT
        //WriteInQueue(&q_messageToMon, msg_to_mon);
    }
}

void Tasks::EnvoiImg(void *arg){
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    bool cam_openned_local = false;
    bool searching_arena_local = false;
    bool use_arena_local = false;
    bool calculate_position = false;
    MessageImg * flux_video;
    rt_task_set_periodic(NULL, TM_NOW, 100000000);
    while (1) {
        rt_task_wait_period(NULL);
        rt_mutex_acquire(&mutex_CamOpened, TM_INFINITE);
        cam_openned_local = CamOpened;
        rt_mutex_release(&mutex_CamOpened);
        rt_mutex_acquire(&mutex_SearchingArena, TM_INFINITE);
        searching_arena_local = SearchingArena;
        rt_mutex_release(&mutex_SearchingArena);
        rt_mutex_acquire(&mutex_SearchingArena, TM_INFINITE);
        use_arena_local = UseArena;
        rt_mutex_release(&mutex_SearchingArena);
        if (!searching_arena_local) {
            if(cam_openned_local) {
                if (use_arena_local) {
                    rt_mutex_acquire(&mutex_Camera, TM_INFINITE);
                    img = new Img(cam->Grab());
                    if (calculate_position) {
                        rt_mutex_acquire(&mutex_RobotPosition, TM_INFINITE); 
                        img->DrawRobot(RobotPosition);
                        rt_mutex_release(&mutex_RobotPosition);
                    }
                    /*ImageMat *img_cropped=img->CropArena(ArenaResult);
                    rt_mutex_release(&mutex_Camera);
                    MessageImg * msgImg = new MessageImg(MESSAGE_CAM_IMAGE, img_cropped);*/
                    img->DrawArena(ArenaResult);
                    flux_video = new MessageImg(MESSAGE_CAM_IMAGE, img);
                    rt_mutex_release(&mutex_Camera);
                }
                else {
                    rt_mutex_acquire(&mutex_Camera, TM_INFINITE);
                    img = new Img(cam->Grab());
                    flux_video = new MessageImg(MESSAGE_CAM_IMAGE, img);
                    rt_mutex_release(&mutex_Camera);
                }
                WriteInQueue(&q_messageToMon, flux_video);
                
            }
            
        }
        

        
    }
}

void Tasks::SearchMyArena(void *arg){
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    bool searching_arena_local = false;
    bool arena_valid_local = false;
    Message *msg_to_mon;
    MessageImg *msgImg;
    Arena arena_local;
    while (1) {
        rt_mutex_acquire(&mutex_SearchingArena, TM_INFINITE);
        searching_arena_local = SearchingArena;
        arena_valid_local = ArenaValid;
        rt_mutex_release(&mutex_SearchingArena);
        if (arena_valid_local) {
            rt_mutex_acquire(&mutex_UseArena, TM_INFINITE);
            UseArena = true;
            rt_mutex_release(&mutex_UseArena);
            rt_mutex_acquire(&mutex_SearchingArena, TM_INFINITE);
            SearchingArena = false;
            rt_mutex_release(&mutex_SearchingArena);
        }
        else if (searching_arena_local) {
            rt_mutex_acquire(&mutex_Camera, TM_INFINITE);
            img = new Img(cam->Grab());
            arena_local = img->SearchArena();
            rt_mutex_release(&mutex_Camera);
                     
            if(arena_local.IsEmpty())
            {
                msg_to_mon = new Message(MESSAGE_ANSWER_NACK);
                WriteInQueue(&q_messageToMon, msg_to_mon);
            }
            else {
                rt_mutex_acquire(&mutex_ArenaResult, TM_INFINITE);
                ArenaResult = arena_local;
                rt_mutex_release(&mutex_ArenaResult);   
                rt_mutex_acquire(&mutex_Camera, TM_INFINITE);
                img = new Img(cam->Grab());
                img->DrawArena(arena_local);
                msgImg = new MessageImg(MESSAGE_CAM_IMAGE, img);
                rt_mutex_release(&mutex_Camera);
                WriteInQueue(&q_messageToMon, msgImg);
            }
        }
        else {
            rt_mutex_acquire(&mutex_SearchingArena, TM_INFINITE);
            SearchingArena = false;
            rt_mutex_release(&mutex_SearchingArena);
            rt_mutex_acquire(&mutex_UseArena, TM_INFINITE);
            UseArena = false;
            rt_mutex_release(&mutex_UseArena);
        }
    }
}

void Tasks::CalculatePosition(void *arg){
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    bool calculate_position = false;
    bool use_arena_local = false;
    std::list<Position> robot_position_list_local;
    MessagePosition * msgPos;
    rt_task_set_periodic(NULL, TM_NOW, 100000000);
    while (1) {
        rt_task_wait_period(NULL);
        rt_mutex_acquire(&mutex_CalculPosition, TM_INFINITE);
        calculate_position=CalculPosition;
        rt_mutex_release(&mutex_CalculPosition);
        rt_mutex_acquire(&mutex_UseArena, TM_INFINITE);
        use_arena_local = UseArena;
        rt_mutex_release(&mutex_UseArena);
        if (calculate_position && use_arena_local) {
            rt_mutex_acquire(&mutex_Camera, TM_INFINITE);
            img = new Img(cam->Grab());
            rt_mutex_acquire(&mutex_ArenaResult, TM_INFINITE);
            robot_position_list_local = img->SearchRobot(ArenaResult);
            rt_mutex_release(&mutex_ArenaResult); 
            
            rt_mutex_release(&mutex_Camera);
            if(robot_position_list_local.empty())
            {
                // pas sûr de celui-la
                rt_mutex_acquire(&mutex_RobotPosition, TM_INFINITE); 
                RobotPosition.center=cv::Point2f(-1.0,-1.0);
                msgPos = new MessagePosition(MESSAGE_CAM_POSITION, RobotPosition);
                rt_mutex_release(&mutex_RobotPosition);
            }
            else {
                rt_mutex_acquire(&mutex_RobotPosition, TM_INFINITE); 
                RobotPosition = robot_position_list_local.front();
                msgPos = new MessagePosition(MESSAGE_CAM_POSITION, RobotPosition);
                rt_mutex_release(&mutex_RobotPosition);
            }
        }
    }
}

