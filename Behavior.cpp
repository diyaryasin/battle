#include "Behavior.h"
#include "ColorFinder.h"

using namespace Robot;

Behavior* Behavior::m_UniqueInstance = new Behavior();

Behavior::Behavior()
{
	State = TRACKING_STATE;

	Tracker = BallTracker();
	Follower = BallFollower();

	CountToReady = 0;
	WaitUntilReady = false;
}

double panLastSeenPost(1);
extern int countHeadSearchingCycle;

Point2D post_pos(-1,-1);
double panCheck;

void Behavior::Process(Point2D center)
{
    ObjectPos = center;

    switch(State)
    {
		case TRACKING_STATE : Tracking(); break;
		case NORMAL_STATE : Normal(); break;
		case CHECK_BALL_MOVEMENT_STATE : CheckBallMovement(); break;
		case PENALTY_KICK_STATE : PenaltyKick(); break;
		case CHECK_COMPASS_STATE : CheckCompass(); break;
		case TURN_AROUND_BALL_STATE : TurnAroundBall(); break;
		case READY_KICK_STATE : ReadyKick(); break;
		case READY_STATE : Ready(); break;
		case HEADING_FRONT_STATE : HeadingFront(); break;
		case WALK1_STATE : Walk1(); break;
		case GO_BACK_STATE : GoBack(); break;
		case FALL_READY_STATE : FallReady(); break;
		case BALL_FACING_STATE : BallFacing(); break;
		case CHECK_HEADING_STATE : CheckHeading(); break;
    }
}

void Behavior::CheckStatus()
{
	switch(State)
    {
		case TRACKING_STATE : printf("TRACKING STATE\n"); break;
		case NORMAL_STATE : printf("NORMAL STATE\n"); break;
		case CHECK_BALL_MOVEMENT_STATE : printf("CHECK BALL MOVEMENT STATE\n"); break;
		case PENALTY_KICK_STATE : printf("PENALTY KICK STATE\n"); break;
		case CHECK_COMPASS_STATE : printf("CHECK COMPASS STATE\n"); break;
		case TURN_AROUND_BALL_STATE : printf("TURN AROUND BALL STATE\n"); break;
		case READY_KICK_STATE : printf("READY KICK STATE\n"); break;
		case READY_STATE : printf("READY STATE\n"); break;
		case HEADING_FRONT_STATE : printf("HEADING FRONT STATE\n"); break;
		case WALK1_STATE : printf("WALK1 STATE\n"); break;
		case GO_BACK_STATE : printf("GO BACK STATE\n"); break;
		case FALL_READY_STATE : printf("FALL READY STATE\n"); break;
		case BALL_FACING_STATE : printf("BALL FACING STATE\n"); break;		
		case CHECK_HEADING_STATE : printf("CHECK HEADING STATE\n"); break;		
    }
}

void Behavior::Tracking()
{
    //Follower.FallingDecision(ObjectPos);

	if(WaitUntilReady)
	{
		if(CountToReady > MaxCountToReady)
		{
			WaitUntilReady = false;
			CountToReady = 0;
		}
		else
		{
		    	Head::GetInstance()->MoveByAngle(0,26);
			CountToReady++;
		}
	}
	else
	{
	    Follower.Tracking(ObjectPos);
	}
}

void Behavior::Normal()
{
	Tracker.Process(ObjectPos);
	Follower.Process(Tracker.ball_position);
}

void Behavior::CheckBallMovement()
{
    Tracker.Process(ObjectPos);
    Follower.ProcessToSaveBall(ObjectPos);
}

void Behavior::PenaltyKick()
{
    Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
    Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);

    Follower.Penalty(ObjectPos);
}

void Behavior::CheckCompass()
{
    Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
    Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
    
    Head::GetInstance()->MoveByAngle(0,40);

	Walking::GetInstance()->X_MOVE_AMPLITUDE = -2;
    if(WaitUntilReady)
    {
	printf("WaitUntilReady\n");
        if(CountToReady > MaxCountToReady + 10)
        {
            WaitUntilReady = false;
            CountToReady = 0;
        }
        else
        {
            CountToReady++;
            panLastSeenPost = 0;
        }
    }
	else
    {
        panLastSeenPost = ObjectPos.X;
		printf("panlastseen post = %f \n",panLastSeenPost);
		if (ObjectPos.X < 45 && ObjectPos.X > -45)
        {
            Walking::GetInstance()->stop_counting_step();

            set_ready_kick_state();

            Head::GetInstance()->MoveByAngle(0, -25);
            CountToReady = 0;
            WaitUntilReady = true;
            printf("DAERAH AMAN -- KICK\n");
        }
        else if ((ObjectPos.X > 45 && ObjectPos.X < 90) || (ObjectPos.X < -45 && ObjectPos.X > -90))
        {
            Walking::GetInstance()->start_counting_step();

            StepToDo = 2* panLastSeenPost/15;
            if(StepToDo < 0) StepToDo = -StepToDo;

            set_turn_around_ball_state();

            Head::GetInstance()->MoveByAngle(0, -25);
            CountToReady = 0;
            WaitUntilReady = true;
            printf("DAERAH SEMI AMAN -- TURN AROUND BALL\n");
        }
		else
		{
            Walking::GetInstance()->start_counting_step();
            StepToDo = 15;

            set_turn_around_ball_state();

            Head::GetInstance()->MoveByAngle(0, -25);
            CountToReady = 0;
            WaitUntilReady = true;
            printf("TOO FAR  -- TURN AROUND BALL\n");
		}
    }
}

void Behavior::TurnAroundBall()
{
	printf("STEPTODO = %d  STEPDOING = %d\n",StepToDo,Walking::GetInstance()->get_walking_step());
                if(WaitUntilReady)
                {
                        if(CountToReady > MaxCountToReady)
                        {
                                WaitUntilReady = false;
                                CountToReady = 0;
                        }
			else
			{
				CountToReady++;
			}
                }
                else
                {
                        if(Walking::GetInstance()->get_walking_step() > StepToDo)
                        {
				Walking::GetInstance()->stop_counting_step();

				//check_post_position();
				set_check_compass_state();

                                Head::GetInstance()->MoveByAngle(0, 60);
                                Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
                                Walking::GetInstance()->Y_MOVE_AMPLITUDE = 0;
                                Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;

				CountToReady = 0;
                        	WaitUntilReady = true;
				printf("Wait true\n");
                        }
                        else
                        {
                                Tracker.Process(ObjectPos);
                                Follower.ProcessTurnAroundBall(Tracker.ball_position);

				if(State == NORMAL_STATE)
				{
					Walking::GetInstance()->stop_counting_step();

					CountToReady = 0;
                                	WaitUntilReady = true;
				}
	                }
                }
}

void Behavior::ReadyKick()
{
    Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
    Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Tracker.ProcessToKick(ObjectPos);

	if(WaitUntilReady)
	{
		if(CountToReady > MaxCountToReady)
		{
			WaitUntilReady = false;
			CountToReady = 0;
		}
		else
		{
			CountToReady++;
		}
	}
	else
	{
		Follower.ProcessToKick(Tracker.ball_position);

		 if(Follower.KickBall != 0)
                 {
			 if (CountToReady > MaxCountToReady)
			 {

        	           Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
                	   Action::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);

                    	    if(Follower.KickBall == -1)
                    	    {
                        	Action::GetInstance()->Start(113);   // RIGHT KICK
	                        fprintf(stderr, "RightKick! \n");
				while(Action::GetInstance()->IsRunning()) usleep(8*1000);
        	            }
                	    else if(Follower.KickBall == 1)
	                    {
        	                Action::GetInstance()->Start(117);   // LEFT KICK
                	        fprintf(stderr, "LeftKick! \n");
				while(Action::GetInstance()->IsRunning()) usleep(8*1000);
                    	    }
				printf("udah nendang, ganti state\n");
			    	//set_heading_front_state();
			    Follower.KickBall = 0;
			    CountToReady = 0;
                	 }
			 else
			 {
				CountToReady++;
			 }
		  }
	}
}

void Behavior::Ready()
{
	Follower.InitKickBallCount();

    set_tracking_state();
     // Behavior::GetInstance()->check_post_heading();

    Walking::GetInstance()->stop_counting_step();

	Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
	Walking::GetInstance()->Y_MOVE_AMPLITUDE = 0;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
	Walking::GetInstance()->A_MOVE_AIM_ON = false;

    Behavior::GetInstance()->CountToReady = 0;
    Behavior::GetInstance()->WaitUntilReady = true;
}

void Behavior::HeadingFront()
{
    Head::GetInstance()->MoveByAngle(0,40);
    Follower.Heading(ObjectPos.X, ObjectPos.Y);
}

void Behavior::Walk1()
{
    Follower.Walk1();
}

void Behavior::GoBack()
{
    Head::GetInstance()->MoveByAngle(0,-5);
    Follower.GoBack(ObjectPos);
}

void Behavior::FallReady()
{
    Head::GetInstance()->MoveByAngle(0,26);
    Follower.FallReady(ObjectPos);    
}

void Behavior::BallFacing()
{
    Tracker.Process(ObjectPos);
	if(WaitUntilReady)
	{
		if(CountToReady > MaxCountToReady)
		{
			WaitUntilReady = false;
			CountToReady = 0;
		}
		else
		{
			CountToReady++;
		}
	}
	else
	{
   		Follower.BallFacing(ObjectPos);    
	}
}

void Behavior::CheckHeading()
{
    Head::GetInstance()->MoveByAngle(0,40);
	if(WaitUntilReady)
	{
		if(CountToReady > MaxCountToReady+15)
		{
			WaitUntilReady = false;
			CountToReady = 0;
		}
		else
		{
			CountToReady++;
		}
	}
	else
	{
   		Follower.set_HeadingAngle(ObjectPos.X);
		Follower.X_step_counter = 0;
		set_normal_state();
		ChangeState();
	}
}


