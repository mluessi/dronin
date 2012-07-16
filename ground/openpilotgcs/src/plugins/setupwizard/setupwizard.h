/**
 ******************************************************************************
 *
 * @file       setupwizard.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2012.
 * @addtogroup GCSPlugins GCS Plugins
 * @{
 * @addtogroup Setup Wizard  Plugin
 * @{
 * @brief A Wizards to make the initial setup easy for everyone.
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#ifndef SETUPWIZARD_H
#define SETUPWIZARD_H

#include <QWizard>

class SetupWizard : public QWizard
{
    Q_OBJECT

public:
    SetupWizard(QWidget *parent = 0);
    int nextId() const;
    enum CONTROLLER_TYPE {CONTROLLER_UNKNOWN, CONTROLLER_CC, CONTROLLER_CC3D, CONTROLLER_REVO, CONTROLLER_PIPX};
    enum VEHICLE_TYPE {VEHICLE_UNKNOWN, VEHICLE_MULTI, VEHICLE_FIXEDWING, VEHICLE_HELI, VEHICLE_SURFACE};
    enum MULTI_ROTOR_SUB_TYPE {MULTI_ROTOR_UNKNOWN, MULTI_ROTOR_TRI_Y, MULTI_ROTOR_QUAD_X, MULTI_ROTOR_QUAD_PLUS,
                               MULTI_ROTOR_HEXA, MULTI_ROTOR_HEXA_H, MULTI_ROTOR_HEXA_COAX_Y, MULTI_ROTOR_OCTO,
                               MULTI_ROTOR_OCTO_V, MULTI_ROTOR_OCTO_COAX_X, MULTI_ROTOR_OCTO_COAX_PLUS};

    void setControllerType(SetupWizard::CONTROLLER_TYPE type) { m_controllerType = type; }
    SetupWizard::CONTROLLER_TYPE getControllerType() const { return m_controllerType; }

    void setVehicleType(SetupWizard::VEHICLE_TYPE type) { m_vehicleType = type; }
    SetupWizard::VEHICLE_TYPE getVehicleType() const { return m_vehicleType; }

private:
    enum {PAGE_START, PAGE_CONTROLLER, PAGE_VEHICLES, PAGE_MULTI, PAGE_FIXEDWING,
          PAGE_HELI, PAGE_SURFACE, PAGE_NOTYETIMPLEMENTED, PAGE_END};
    void createPages();

    CONTROLLER_TYPE m_controllerType;
    VEHICLE_TYPE m_vehicleType;
};

#endif // SETUPWIZARD_H
