from flask import Flask, jsonify, render_template
from mapping import mapping

app = Flask(__name__)


@app.route("/")
def accueil():
    return render_template("index.html")


@app.route("/api/mapping", methods=["GET"])
def obtenir_mapping():
    return jsonify(mapping.obtenir_donnees())


@app.route("/api/mapping/demarrer", methods=["POST"])
def demarrer_mapping():
    mapping.demarrer()
    return jsonify({"message": "Scan demandé"})


@app.route("/api/mapping/arreter", methods=["POST"])
def arreter_mapping():
    mapping.arreter()
    return jsonify({"message": "Arrêt demandé"})


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=False)