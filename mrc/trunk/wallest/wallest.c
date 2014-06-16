/***************************************************************************
                          regression.c  -  description
                             -------------------
    begin                : Tue Dec 16 2003
    copyright            : (C) 2003 by Jose Fernandez,,,
    email                : ex04@little
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#include "wallest.h"

void wall_coordinates(wallesttype *regr);
// side, measurement , odox, odoy, odoth,
// out x and y;
// memory not used

void distance(wallesttype *regr);
// a, b, c, x, y    * a, b ,c and d from memory.
// out error_distance and wall_distance


void detect_hole(wallesttype *regr);
// in distance
// out wall, number of doors, add_data
// memory, wall, number of doors, good_points, missed_points

void regression_add(wallesttype *regr);
// in x, y
// out a, b, c, angle
// memory sx, sy, pxy, sx2, sy2, xdata, ydata, length, initialized_part, next_position.

void regression_reset(wallesttype *regr);
// reset, input_length
// out a, b, c, angle
// memory sx, sy, pxy, sx2, sy2, xdata, ydata, length, initialized_part, next_position.

void regression_init(wallesttype *regr);
// it put the default parameter values in the apropiate variables
void regression_update(wallesttype *regr);


//next line for debuging
void close_debug(wallesttype *regr);
void init_debug(wallesttype *regr);
void debug(wallesttype *regr);






void wall_estimation(wallesttype *regr){
  if(regr->status == WE_ON){
    if(regr->previous_status == WE_OFF){
      regression_init(regr);
      regression_reset(regr);
//next line for debuging
			init_debug(regr);
    }
	  regression_update(regr);
  }
//next line for debuging
  if(regr->status == WE_OFF && regr->previous_status==WE_ON) { close_debug(regr); puts("clossing debug");}
  regr->previous_status = regr->status;
//  if(regr->new_distance_valuel) printf("fwddist=%f odox=%f odoy=%f odoth=%f wdist=%f errdist=%f x=%f y=%f a=%f b=%f c=%f iswall=%d
// 
// holes=%d\n",regr->fwddist,regr->odox,regr->odoy,regr->odoth,regr->wall_distance,regr->error_distance,regr->x,regr->y,regr->a,regr->b,regr->c,regr->is_wall,regr->holes);

}

void regression_update(wallesttype *regr){
    if((regr->new_distance_valuel && regr->side == WE_LEFT) ||
       (regr->new_distance_valuer && regr->side == WE_RIGHT)
    ){
      wall_coordinates(regr); //calculating point coordinates from the measured data.
      if(regr->initialized_part < regr->length){
//next line for debuging
        if(regr->reliant) regr->error_distance = fabsf((regr->a * regr->x + regr->b * regr->y + regr->c)/sqrt( pow(regr->a,2) + pow(regr->b,2) ));
        regression_add(regr);
      }else{
        detect_hole(regr);
        if(regr->add_data)regression_add(regr);
      }
//next line for debuging
        if(regr->reliant)distance(regr); //because we want a calculation of distance before debug.
//next line for debuging
        debug(regr);
    }
    if(regr->reliant)distance(regr); //the angle of the regression line is the same if the regression line doesn't change, the distance is not the same, it depends also on the robot position

}

void regression_reset(wallesttype *regr){
/* all the line regression data will be reseted and the length of the line regression will be updated */
  regr->sx = regr->sy = regr->sx2 = regr->sy2 = regr->pxy = 0.0;
  regr->a = regr->b = regr->c = regr->angle = 0.0;
  regr->initialized_part = regr->next_position = 0;
  regr->length = (regr->line_distance / (regr->target_speed * SAMPLE_DISTANCE_TIME));
  printf("Number of points: %d\n",regr->length);
	if(regr->length < MINIMUN_REGRESSION_POINTS) regr->length = MINIMUN_REGRESSION_POINTS;
printf("Number of points: %d\n",regr->length);
  regr->is_wall = 1;
  regr->good_points = 0;
  regr->missed_points = 0;
  regr->holes = 0;
	regr->uncondup = 0;
	regr->reliant = 0;
	regr->lastdist = regr->fwddist;	

  regr->max_hole_dist = regr->input_max_hole_dist;
  regr->min_wall_after_hole = regr->input_min_wall_after_hole;
}

void regression_add(wallesttype *regr){
  float delta_x, delta_y;
  float n;
  /* a new pair of values x and y will be added to the regression line */
  if(regr->initialized_part >= regr->length){
  /* the neccesary number of data has been already stored */
    /* eliminating data from the total amount */
    regr->sx -= (double) regr->xdata[regr->next_position];
    regr->sy -= (double) regr->ydata[regr->next_position];
    regr->sx2 -= (double) (regr->xdata[regr->next_position] * regr->xdata[regr->next_position]);
    regr->sy2 -= (double) (regr->ydata[regr->next_position] * regr->ydata[regr->next_position]);
    regr->pxy -= (double) (regr->xdata[regr->next_position] * regr->ydata[regr->next_position]);
    /* calculating delta_x and delta_y */
    delta_x = regr->x - regr->xdata[regr->next_position];
    delta_y = regr->y - regr->ydata[regr->next_position];
    /* storing new data in the in the data array */
    regr->xdata[regr->next_position] = regr->x;
    regr->ydata[regr->next_position] = regr->y;

    /* updating counters */
    if( (++(regr->next_position)) >= regr->length ) regr->next_position = 0;
  }else{
  /* the neccesary amount of data has not been stored yet */
    /* calculating delta_x and delta_y */
    delta_x = regr->x - regr->xdata[0];
    delta_y = regr->y - regr->ydata[0];
    /* storing new data in the array */
    regr->xdata[regr->initialized_part] = regr->x;
    regr->ydata[regr->initialized_part] = regr->y;

    /* updating counters */
    regr->initialized_part++;
  }

  /*adding data to the total amount */

  regr->sx += (double) regr->x;
  regr->sy += (double) regr->y;
  regr->sx2 += (double) (regr->x * regr->x);
  regr->sy2 += (double) (regr->y * regr->y);
  regr->pxy += (double) (regr->x * regr->y);

  /* calculating a, b and c, coeficients */
  n = (float) regr->initialized_part;   /* It will contain the last initialized_part value increased */
  if( (regr->reliant == 0) && 
      (( n*regr->sx2 - regr->sx*regr->sx )!=0.0 || ( n*regr->sy2 - regr->sy*regr->sy )!=0.0 ) &&
      (regr->fwddist >= (regr->lastdist + regr->reliant_distance))
  )regr->reliant = 1; // the result will have sense and we calculate it
  if(regr->reliant == 1){  
    if( fabsf( n*regr->sx2 - regr->sx*regr->sx ) > fabsf( n*regr->sy2 - regr->sy*regr->sy ) ){
      if(fabsf(n*regr->sx2 - regr->sx*regr->sx) > 0.0){
        regr->a = ( n*regr->pxy - regr->sx*regr->sy ) / (n*regr->sx2 - regr->sx*regr->sx );
        regr->b = - 1.0;
        regr->c = ( regr->sy - regr->a * regr->sx ) / n;
      }
    }else{
      if(fabsf( n*regr->sy2 - regr->sy*regr->sy ) > 0.0){
        regr->a = - 1.0;
        regr->b = ( n*regr->pxy - regr->sx*regr->sy ) / ( n*regr->sy2 - regr->sy*regr->sy );
        regr->c =  (regr->sx - regr->b*regr->sy) / n;
      }
    }

    /* Calculation of the angle of the regression line */
    /* the angle of our regression line in range -pi/2 to 3pi/2 is: */
    regr->angle = (M_PI / 2.0) + atan2f(regr->b,regr->a);
    if( regr->angle > (M_PI) ) regr->angle -=  (2.0* M_PI);
    /* Now regression angle belongs to (-pi,pi) range */
    /* note that the direction is correct but the orientation can be wrong */
    /* Correction of the orientation in angle of the regression line */
    /* for doing this task we will use the values of delta_x and delta_y,
       one value is enough and we will use the greatest of both to avoid
       possible problems such as zero values with lines very close to one of the axis*/

    if(fabsf(delta_x) > fabsf(delta_y)){
      if(delta_x > 0.0){
      /* this means that the angle of the line belong to the first or fourth quadrant
         lets check if it is true and if its not lets correct */
        if(cos(regr->angle) < 0.0) regr->angle += M_PI;
      }else{
      /* this means that the angle of the line belong to the second or third quadrant
         lets check if it is true and if its not lets correct */
        if(cos(regr->angle) > 0.0) regr->angle += M_PI;
      }
    }else{
      if(delta_y > 0.0){
      /* this means that the angle of the line belong to the first or second quadrant
         lets check if it is true and if its not lets correct */
        if(sin(regr->angle) < 0.0) regr->angle += M_PI;
      }else{
      /* this means that the angle of the line belong to the third or fourth quadrant
         lets check if it is true and if its not lets correct */
        if(sin(regr->angle) > 0.0) regr->angle += M_PI;
      }
    }
    /* Normalizing the angle to get a value in the range (-pi,pi) */
    if( regr->angle > (M_PI) ) regr->angle -=  (2.0* M_PI);
  }
}


void wall_coordinates(wallesttype *regr){
  switch(regr->side){
    case WE_RIGHT:
      /* right side */
      regr->x = regr->odox + regr->robot_length * cos( regr->odoth ) + (regr->half_robot_width + regr->measured_distancer) * sin( regr->odoth );
      regr->y = regr->odoy + regr->robot_length * sin( regr->odoth ) - (regr->half_robot_width + regr->measured_distancer) * cos( regr->odoth );
      break;
    case WE_LEFT:
      /* left side */
      regr->x = regr->odox + regr->robot_length * cos( regr->odoth ) - (regr->half_robot_width + regr->measured_distancel) * sin( regr->odoth );
      regr->y = regr->odoy + regr->robot_length * sin( regr->odoth ) + (regr->half_robot_width + regr->measured_distancel) * cos( regr->odoth );
      break;
    default:;
  }
}

void distance(wallesttype *regr){
  regr->wall_distance = fabsf((regr->a * regr->odox + regr->b * regr->odoy + regr->c)/sqrt( pow(regr->a,2) + pow(regr->b,2) ));
}

void detect_hole(wallesttype *regr){

  /* calculating the distance of the new point to the estimated line */
  regr->error_distance = fabsf((regr->a * regr->x + regr->b * regr->y + regr->c)/sqrt( pow(regr->a,2) + pow(regr->b,2) ));
  if ( regr->is_wall == 1 ){
  /* If we are following the wall and we detect a measurement which has a difference of more than 3.5 cm with the estimated line
  we don't use the value beacuase we consider it to be noise, if there are more than three consecutive values like that we will
  consider that there is a hole in the wall and we will start focussing in detecting when the hole finishes*/
    if(regr->uncondup == 1){
      if(regr->fwddist >= (regr->lastdist + regr->min_wall_after_hole))regr->uncondup = 0;
    }
    if(regr->uncondup == 0){
      if( regr->error_distance >= regr->tolerance_for_hole){
        regr->add_data = 0;
        regr->missed_points++;
      }else{
        regr->add_data = 1;  /* It belongs to the wall */
        regr->missed_points = 0;
      }
      if(regr->missed_points == regr->points_for_hole){
        regr->is_wall = 0;
        regr->good_points = 0;
        regr->holes++;
        regr->lastdist = regr->fwddist;
      }
    }
  }else{
    if(regr->error_distance <= regr->tolerance_for_wall){
      regr->good_points++;
    }else{
      regr->good_points = 0;
    }
    regr->add_data = 0;  /* In any case we don't use the data for updating the regression line if we are in a hole */
    if(  (regr->good_points == regr->points_for_hole) ||
         (
           ( regr->fwddist >= (regr->lastdist + regr->max_hole_dist) ) &&
           ( regr->max_hole_dist != 0.0 )
         )
    ){
      regr->is_wall = 1;
      regr->add_data = 1;
      regr->missed_points = 0;
      regr->uncondup = 1;
      regr->lastdist = regr->fwddist;
    }
  }
}

void regression_init(wallesttype *regr){
  regr->points_for_hole = POINTS_FOR_HOLE;
  regr->points_for_wall = POINTS_FOR_WALL;
  regr->tolerance_for_hole = TOLERANCE_FOR_HOLE;
  regr->tolerance_for_wall = TOLERANCE_FOR_WALL;
  regr->robot_length = ROBOT_LENGTH;
  regr->half_robot_width = HALF_ROBOT_WIDTH;
  regr->reliant_distance = RELIANT_DISTANCE;

}


//next functions are for debug


void debug(wallesttype *regr){
//gendeb,regx,regy;
//gendeb, reliant, number of points, ododist, odox, odoy, odoth, a, b, c, angle, walldist, error, is_wall, doors.
  int i;
  fprintf(regr->gendeb,"%d %d %f %f %f %f %f %f %f %f %f %f %d %d\n",regr->reliant,regr->initialized_part,regr->fwddist,regr->odox,regr->odoy,regr->odoth,regr->a,regr->b,regr->c,regr->angle,regr->wall_distance,regr->error_distance,regr->is_wall,regr->holes);

  for(i=0;i<regr->length;i++){
    fprintf(regr->regx,"%f ",regr->xdata[i]);
  }
  fprintf(regr->regx,"\n");

  for(i=0;i<regr->length;i++){
    fprintf(regr->regy,"%f ",regr->ydata[i]);
  }
  fprintf(regr->regy,"\n");
}


void init_debug(wallesttype *regr){
  regr->gendeb = fopen("gendeb.dat","w");
  regr->regx = fopen("regx.dat","w");
  regr->regy = fopen("regy.dat","w");
}

void close_debug(wallesttype *regr){
  fclose(regr->gendeb);
  fclose(regr->regx);
  fclose(regr->regy);
}

