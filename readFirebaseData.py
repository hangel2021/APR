# -*- coding: utf-8 -*-
"""
Created on Sun Jul  7 20:44:22 2024

@author: Desarrollo
"""

import firebase_admin
from firebase_admin import credentials
from firebase_admin import firestore

import matplotlib.pyplot as plt
from datetime import datetime

path = 'C:/Users/Desarrollo/Desktop/Satori SpA/Cerrados/Constructora Okinawa/CODE/'

cred = credentials.Certificate(path + 'ServiceAccountKey.json')
app = firebase_admin.initialize_app(cred)
firestore_client = firestore.client()
db = firestore.client()
docs = db.collection('PiedraNegra').get()

nivel = []
for d in docs:
    nivel.append(d.to_dict()["nivel"])
    
    
dt = []
for d in docs:
    t = d.to_dict()["ts"]
    dt_object = datetime.fromtimestamp(t.timestamp())
    dt.append(dt_object)
    
