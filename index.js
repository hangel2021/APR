const functions = require("firebase-functions");
const admin = require("firebase-admin");

const express = require("express");
const cors = require("cors");

const http = require("https");

const serviceAccount = require("./ServiceAccountKey.json");

admin.initializeApp({
  credential: admin.credential.cert(serviceAccount),
});

const app = express();

app.use(cors({origin: true}));

// Main database reference
const db = admin.firestore();

// Routes
app.get("/", (req, res) => {
  return res.status(200).send("Test APRMon app");
});

app.post("/api/create/PiedraNegra", async (req, res) => {
  console.log(req.body);
  try {
      let val = req.body; 

      let tipo = val.tipo;
      let ts = new Date(val.ts * 1000);
      let id = val.id;
      let rssi = val.rssi;
      let IMEI = val.IMEI;
      let ICCID = val.ICCID;
      let gp = new admin.firestore.GeoPoint(val.GPS.lat, val.GPS.lon);
    
      if(tipo == "Normal")
      {
        let nivel = val.data.nivel;
        let LoRaTx = val.data.LoRaTx;
        let LoRaRx = val.data.LoRaRx;
        let txRSSI = val.data.txRSSI;
        let rxRSSI = val.data.rxRSSI;
        let bOnOff = val.data.bOnOff;
        let modo = val.data.modo;

        let voltA = val.data.pElect.voltA;
        let voltB = val.data.pElect.voltB;
        let voltC = val.data.pElect.voltC;
        let ampA = val.data.pElect.ampA;
        let ampB = val.data.pElect.ampB;
        let ampC = val.data.pElect.ampC;
        let actPowA = val.data.pElect.actPowA;
        let actPowB = val.data.pElect.actPowB;
        let actPowC = val.data.pElect.actPowC;
        let reactPowA = val.data.pElect.reactPowA;
        let reactPowB = val.data.pElect.reactPowB;
        let reactPowC = val.data.pElect.reactPowC;
        let powFA = val.data.pElect.powFA;
        let powFB = val.data.pElect.powFB;
        let powFC = val.data.pElect.powFC;
        let freqz = val.data.pElect.freqz;
        let totP = val.data.pElect.totP;
        let totQ = val.data.pElect.totQ;

        let events = val.events;

              // Insertar valor a Data
        await db.collection("PiedraNegra").doc(`/${Date.now()}/`).create({
              id: id,
              ts: ts,
              rssi: rssi,
              IMEI: IMEI,
              ICCID: ICCID,
              gp: gp,
              nivel: nivel,
              LoRaTx: LoRaTx,
              LoRaRx: LoRaRx,
              txRSSI: txRSSI,
              rxRSSI: rxRSSI,
              bOnOff: bOnOff,
              modo: modo,
              pElect:
              {
                  voltA: voltA,
                  voltB: voltB,
                  voltC: voltC,
                  ampA: ampA,
                  ampB: ampB,
                  ampC: ampC,
                  actPowA: actPowA,
                  actPowB: actPowB,
                  actPowC: actPowC,
                  reactPowA: reactPowA,
                  reactPowB: reactPowB,
                  reactPowC: reactPowC,
                  powFA: powFA,
                  powFB: powFB,
                  powFC: powFC,
                  freqz: freqz,
                  totP: totP,
                  totQ: totQ
              },
              
              events: events
        });
      
        return res.status(200).send({ status: "Success", msg: "Data Saved" });
      }

      else if(tipo == "Alarma")
      {
        let alarma = val.data.alarma;

        await db.collection("Alarmas").doc(`/${Date.now()}/`).create({
            id: id,
            ts: ts,
            rssi: rssi,
            IMEI: IMEI,
            ICCID: ICCID,
            gp: gp,
            alarma: alarma

        });       
        return res.status(200).send({ status: "Success", msg: "Data Saved" });
      }
  
  } catch(error) { 
      console.log(error);
      res.status(500).send({ status: "Failed", msg: error });
  }
});

// exports the api to firebase cloud functions
exports.app = functions.https.onRequest(app);
