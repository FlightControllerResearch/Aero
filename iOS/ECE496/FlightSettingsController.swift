//
//  DeliverySettingsController.swift
//  D2U
//
//  Created by Luke Armbruster on 12/23/18.
//  Copyright © 2018 Luke Armbruster. All rights reserved.
//

import MapKit
import UIKit

class FlightSettingsController: UIViewController {
    
    @IBOutlet weak var deliverBtn: UIButton!
    
    @IBOutlet weak var currentLocationLbl: UILabel!
    @IBOutlet weak var currentLandmarkLbl: UILabel!
    @IBOutlet weak var currentDistanceLbl: UILabel!
    @IBOutlet weak var etaLabel: UILabel!
    
    var syncTimer: Timer!
    
    private func disableDeliveryBtn() {
        deliverBtn.isEnabled = false
        deliverBtn.alpha = 0.5
    }
    
    private func enableDeliveryBtn() {
        deliverBtn.isEnabled = true
        deliverBtn.alpha = 1.0
    }
    
    @IBAction func deliverBtn(_ sender: UIButton) {
        disableDeliveryBtn()
        ref.updateChildValues(["Delivery Requested" : true]) // Signal the device user requests delivery
    }
    
    private func handleViewingEvents() {
        if FlightInformation.flightInformation.delivering {
            self.disableDeliveryBtn()
        }
        self.currentLocationLbl.text = String(format: "%.4f, %.4f", FlightInformation.flightInformation.curCoord?.latitude ?? 0.0, FlightInformation.flightInformation.curCoord?.longitude ?? 0.0)
        self.currentLandmarkLbl.text = FlightInformation.flightInformation.nearestLandmark
        self.currentDistanceLbl.text = FlightInformation.flightInformation.distanceRemaining
        self.etaLabel.text = FlightInformation.flightInformation.eta
    }
    
    @objc func sync() {
        handleViewingEvents()
    }
    
    override func viewDidLoad() {
        handleViewingEvents()
        syncTimer = Timer.scheduledTimer(timeInterval: 1.0, target: self, selector: #selector(self.sync), userInfo: nil, repeats: true) // Update UI stuff about every second
    }
}
