/********************************************************************************
Copyright (c) 2016, TRACLabs, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice,
       this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.

    3. Neither the name of the copyright holder nor the names of its
       contributors may be used to endorse or promote products derived from
       this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************/

#include <rviz_affordance_template_panel/affordance.h>

#define PIXMAP_SIZE 100
#define CLASS_INDEX 0
#define TRAJECTORY_DATA 1
#define IMAGE 2
#define FILENAME 3
#define DISPLAY_OBJECTS 4

using namespace rviz_affordance_template_panel;

Affordance::Affordance(const string& class_type, const string& image_path, QMap<QString, QVariant> &trajectory_map, QStringList &display_objects, const string& filename) {
    QPixmap pixmap(QSize(PIXMAP_SIZE,PIXMAP_SIZE));

    // set pixmap to image if it exists
    pixmap.convertFromImage(QImage(image_path.c_str()));
    if (pixmap.isNull()) {
        // otherwise set it to a green box with the class name overlayed
        QPixmap colorPixmap(QSize(PIXMAP_SIZE,PIXMAP_SIZE));
        colorPixmap.fill(Qt::green);
        QPainter text(&colorPixmap);
        QRectF rect(0, 0, PIXMAP_SIZE, PIXMAP_SIZE);
        text.drawText(rect, Qt::AlignCenter, class_type.c_str());
        this->setPixmap(colorPixmap.scaled(PIXMAP_SIZE, PIXMAP_SIZE));
    } else {
        this->setPixmap(pixmap.scaled(PIXMAP_SIZE, PIXMAP_SIZE));
    }

    this->setFlag(QGraphicsItem::ItemIsSelectable);

    // store the class name associated with the template pixmap item
    // we'll use the class name to instantiate an object template using Pluginlib
    this->setData(CLASS_INDEX, QVariant(class_type.c_str()));
    this->setData(TRAJECTORY_DATA, QVariant(trajectory_map));
    this->setData(IMAGE, QVariant(image_path.c_str()));
    this->setData(FILENAME, QVariant(filename.c_str()));
    this->setData(DISPLAY_OBJECTS, QVariant(display_objects));
    
    this->key_ = class_type;
    this->map_ = trajectory_map;
    this->objs_ = display_objects;
}