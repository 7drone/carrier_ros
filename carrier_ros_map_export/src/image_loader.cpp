/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * This file contains helper functions for loading images as maps.
 *
 * Author: Brian Gerkey
 */

#include <cstring>
#include <stdexcept>

#include <stdlib.h>
#include <stdio.h>
#include "ros/ros.h"
// We use SDL_image to load the image from disk
#include <SDL/SDL_image.h>

// Use Bullet's Quaternion object to create one from Euler angles
#include <LinearMath/btQuaternion.h>

#include "carrier_ros_map_export/image_loader.h"

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

namespace map_server
{

void
loadMapFromFile(nav_msgs::GetMap::Response* resp,
                const char* fname, double res, bool negate,
                double occ_th, double free_th, double* origin,
                MapMode mode)
{
  SDL_Surface* img;

  unsigned char* pixels;
  unsigned char* p;
  unsigned char value;
  int rowstride, n_channels, avg_channels;
  unsigned int i,j;
  int k;
  double occ;
  int alpha;
  int color_sum;
  double color_avg;
  int x_min, x_max, y_min, y_max;
  int RED, BLUE, GREEN;
  x_min = 0;
  x_max = 0;
  y_min = 0;
  y_max = 0;
  // Load the image using SDL.  If we get NULL back, the image load failed.
  if(!(img = IMG_Load(fname)))
  {
    std::string errmsg = std::string("failed to open image file \"") +
            std::string(fname) + std::string("\": ") + IMG_GetError();
    throw std::runtime_error(errmsg);
  }

  // Copy the image data into the map structure
  resp->map.info.width = img->w-x_min-x_max;
  resp->map.info.height = img->h-y_min-y_max;
  resp->map.info.resolution = res;
  resp->map.info.origin.position.x = *(origin);
  resp->map.info.origin.position.y = *(origin+1);
  resp->map.info.origin.position.z = 0.0;
  btQuaternion q;
  // setEulerZYX(yaw, pitch, roll)
  q.setEulerZYX(*(origin+2), 0, 0);
  resp->map.info.origin.orientation.x = q.x();
  resp->map.info.origin.orientation.y = q.y();
  resp->map.info.origin.orientation.z = q.z();
  resp->map.info.origin.orientation.w = q.w();

  // Allocate space to hold the data
  resp->map.data.resize(resp->map.info.width * resp->map.info.height);

  // Get values that we'll need to iterate through the pixels
  rowstride = img->pitch;
  n_channels = img->format->BytesPerPixel;
  // NOTE: Trinary mode still overrides here to preserve existing behavior.
  // Alpha will be averaged in with color channels when using trinary mode.
  if (mode==TRINARY || !img->format->Amask)
    avg_channels = n_channels;
  else
    avg_channels = n_channels - 1;

  ROS_INFO("avg_channels : %d",avg_channels);
  // Copy pixel data into the map structure
  pixels = (unsigned char*)(img->pixels);
  ROS_INFO("%hhn",pixels);
  for(j = y_min; j < resp->map.info.height-y_max; j++)
  {
    for (i = x_min; i < resp->map.info.width-x_max; i++)
    {
      // Compute mean of RGB for this pixel
      p = pixels + j*rowstride + i*n_channels;
      if(avg_channels>2)
      {
        color_sum = 0;
        RED = *p;
        GREEN = *(p+1);
        BLUE = *(p+2);
        ROS_INFO_ONCE("RED : %d, blue:%d, green:%d",RED,BLUE,GREEN);
        // ROS_INFO("RED : %d, blue:%d, green:%d",RED,BLUE,GREEN);
        if (RED >= 245 || BLUE >= 226)
        {
          resp->map.data[MAP_IDX(resp->map.info.width,i,resp->map.info.height - j - 1)] = 0;
        }
        else
        {
          resp->map.data[MAP_IDX(resp->map.info.width,i,resp->map.info.height - j - 1)] = 100;
        }
      }
      else
      {
        for(k=0;k<avg_channels;k++)
          color_sum += *(p + (k));
        color_avg = color_sum / (double)avg_channels;
      
        if (n_channels == 1)
            alpha = 1;
        else
            alpha = *(p+n_channels-1);

        if(negate)
          color_avg = 255 - color_avg;

        if(mode==RAW){
            value = color_avg;
            resp->map.data[MAP_IDX(resp->map.info.width,i,resp->map.info.height - j - 1)] = value;
            continue;
        }


        // If negate is true, we consider blacker pixels free, and whiter
        // pixels occupied.  Otherwise, it's vice versa.
        occ = (255 - color_avg) / 255.0;

        // Apply thresholds to RGB means to determine occupancy values for
        // map.  Note that we invert the graphics-ordering of the pixels to
        // produce a map with cell (0,0) in the lower-left corner.
        if(occ > occ_th)
          value = +100;
        else if(occ < free_th)
          value = 0;
        else if(mode==TRINARY || alpha < 1.0)
          value = -1;
        else {
          double ratio = (occ - free_th) / (occ_th - free_th);
          value = 1 + 98 * ratio;
        }

        resp->map.data[MAP_IDX(resp->map.info.width,i,resp->map.info.height - j - 1)] = value;
      }
    }
  }
  
  SDL_FreeSurface(img);
}

}