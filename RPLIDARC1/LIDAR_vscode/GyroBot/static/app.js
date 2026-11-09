// GYROBOT --> interface web    
//____________________________________________


const API_MAPPING = "/api/mapping";

let dernieresDonnees = null;
let derniereErreurReseau = null;


//1 Navigation entre les onglets


document.querySelectorAll(".onglet").forEach((bouton) => {
    bouton.addEventListener("click", () => {
        const pageCible = bouton.dataset.page;

        document.querySelectorAll(".onglet").forEach((onglet) => {
            onglet.classList.remove("actif");
        });

        document.querySelectorAll(".page").forEach((page) => {
            page.hidden = page.id !== pageCible;
        });

        bouton.classList.add("actif");
    });
});



//2 Préparation de la zone à scannerr

const zoneCarte = document.getElementById("zone-carte");

let canvas = null;
let contexte = null;

if (zoneCarte) {
    zoneCarte.innerHTML = "";

    canvas = document.createElement("canvas");
    canvas.id = "canvas-lidar";
    canvas.style.display = "block";
    canvas.style.width = "100%";
    canvas.style.height = "auto";
    canvas.style.aspectRatio = "1 / 1";

    zoneCarte.appendChild(canvas);
    contexte = canvas.getContext("2d");
}

function redimensionnerCanvas() {
    const largeur = 1000;
    const hauteur = 1000;

    // La taille du dessin suit la taille de la zone
    const ratio = window.devicePixelRatio || 1;

    canvas.width = largeur * ratio;
    canvas.height = hauteur * ratio;
    canvas.style.width = `${largeur}px`;
    canvas.style.height = `${hauteur}px`;

    contexte.setTransform(ratio, 0, 0, ratio, 0, 0);

    if (dernieresDonnees) {
        dessinerScan(dernieresDonnees.points || []);
    }
}

window.addEventListener("resize", redimensionnerCanvas);
redimensionnerCanvas();


//3 Dessin du scan LiDAR

function dessinerScan(points) {
    if (!canvas || !contexte) return;

    const largeur = canvas.clientWidth;
    const hauteur = canvas.clientHeight;

    const centreX = largeur / 2;
    const centreY = hauteur / 2;

    const rayonMax = Math.min(largeur, hauteur) * 0.42;
    const porteeMax = 5.0; // mètres

    contexte.clearRect(0, 0, largeur, hauteur);

    // fond
    contexte.fillStyle = "#101820";
    contexte.fillRect(0, 0, largeur, hauteur);

    // cercles de distance
    contexte.strokeStyle = "#344653";
    contexte.lineWidth = 1;
    contexte.fillStyle = "#b8c7d1";
    contexte.font = "11px sans-serif";

    for (let distance = 1; distance <= porteeMax; distance++) {
        const rayon = (distance / porteeMax) * rayonMax;

        contexte.beginPath();
        contexte.arc(centreX, centreY, rayon, 0, Math.PI * 2);
        contexte.stroke();

        contexte.fillText(
            `${distance} m`,
            centreX + 5,
            centreY - rayon + 12
        );
    }

    // axes à afficher
    contexte.beginPath();
    contexte.moveTo(centreX - rayonMax, centreY);
    contexte.lineTo(centreX + rayonMax, centreY);
    contexte.moveTo(centreX, centreY - rayonMax);
    contexte.lineTo(centreX, centreY + rayonMax);
    contexte.stroke();

    // points détectés par le lidar
    contexte.fillStyle = "#36d399";

    for (const point of points) {
        const distance = Number(point.distance);
        const angle = Number(point.angle);

        if (!Number.isFinite(distance) ||
            !Number.isFinite(angle) ||
            distance <= 0 ||
            distance > porteeMax) {
            continue;
        }

        const angleRad = angle * Math.PI / 180;

        const x = centreX +
            Math.cos(angleRad) * distance / porteeMax * rayonMax;

        const y = centreY -
            Math.sin(angleRad) * distance / porteeMax * rayonMax;

        contexte.beginPath();
        contexte.arc(x, y, 2, 0, Math.PI * 2);
        contexte.fill();
    }

    // position du LiDAR au centre
    contexte.fillStyle = "#ffffff";
    contexte.beginPath();
    contexte.arc(centreX, centreY, 4, 0, Math.PI * 2);
    contexte.fill();
}


//4 Mise à jour des informations en temps réel

function afficherDonnees(donnees) {
    dernieresDonnees = donnees;

    const etatConnexion = document.getElementById("etat-connexion");
    const etatLidar = document.getElementById("etat-lidar");
    const etatRobot = document.getElementById("etat-robot");
    const distanceMinimale = document.getElementById("distance-minimale");
    const distanceDevant = document.getElementById("distance-devant");
    const nombreMesures = document.getElementById("nombre-mesures");
    const etatMapping = document.getElementById("etat-mapping");
    const derniereMaj = document.getElementById("derniere-maj");
    const connexionReseau = document.getElementById("connexion-reseau");
    const etatProgramme = document.getElementById("etat-programme");

    const points = Array.isArray(donnees.points)
        ? donnees.points
        : [];

    if (etatConnexion) {
        etatConnexion.textContent = "● Serveur connecté";
    }

    if (etatLidar) {
        etatLidar.textContent = donnees.erreur
            ? "Erreur LiDAR"
            : donnees.actif
                ? "Scan en cours"
                : "En attente";
    }

    if (etatRobot) {
        etatRobot.textContent = "Supervision";
    }

    if (nombreMesures) {
        nombreMesures.textContent = points.length;
    }

    if (etatMapping) {
        etatMapping.textContent = donnees.actif
            ? "Scan LiDAR en cours"
            : "Scan arrêté";
    }

    if (connexionReseau) {
        connexionReseau.textContent = "Connecté au serveur";
    }

    if (etatProgramme) {
        etatProgramme.textContent = donnees.erreur
            ? `Erreur : ${donnees.erreur}`
            : "Fonctionnement normal";
    }

    if (derniereMaj) {
        derniereMaj.textContent =
            new Date().toLocaleTimeString("fr-FR");
    }

    if (points.length > 0) {
        const distances = points
            .map((point) => Number(point.distance))
            .filter((distance) =>
                Number.isFinite(distance) && distance > 0
            );

        if (distances.length > 0 && distanceMinimale) {
            distanceMinimale.textContent =
                `${Math.min(...distances).toFixed(2)} m`;
        }

        // Secteur frontal : angles proches de 0°, convention dépendant de l'orientation du capteur
        const devant = points.filter((point) => {
            const angle = Number(point.angle);
            return angle <= 15 || angle >= 345;
        });

        const distancesDevant = devant
            .map((point) => Number(point.distance))
            .filter((distance) =>
                Number.isFinite(distance) && distance > 0
            );

        if (distanceDevant && distancesDevant.length > 0) {
            distanceDevant.textContent =
                `${Math.min(...distancesDevant).toFixed(2)} m`;
        }
    }

    dessinerScan(points);
}

//5 Communication avec Flask


async function actualiserScan() {
    try {
        const reponse = await fetch(API_MAPPING, {
            method: "GET",
            cache: "no-store"
        });

        if (!reponse.ok) {
            throw new Error(`Erreur HTTP ${reponse.status}`);
        }

        const donnees = await reponse.json();

        derniereErreurReseau = null;
        afficherDonnees(donnees);

    } catch (erreur) {
        if (derniereErreurReseau !== erreur.message) {
            console.error("Connexion au serveur impossible :", erreur);
            derniereErreurReseau = erreur.message;
        }

        const etatConnexion =
            document.getElementById("etat-connexion");

        if (etatConnexion) {
            etatConnexion.textContent = "● Serveur inaccessible";
        }
    }
}


//6 Démarrage et arrêt du scan

async function envoyerCommande(action) {
    try {
        const reponse = await fetch(
            `${API_MAPPING}/${action}`,
            { method: "POST" }
        );

        if (!reponse.ok) {
            throw new Error(`Erreur HTTP ${reponse.status}`);
        }

        await actualiserScan();

    } catch (erreur) {
        console.error(`Impossible de ${action} le scan :`, erreur);
        alert(
            "Impossible d'envoyer la commande. " +
            "Vérifie que le serveur Flask fonctionne."
        );
    }
}

document
    .getElementById("demarrer-mapping")
    ?.addEventListener("click", () => envoyerCommande("demarrer"));

document
    .getElementById("arreter-mapping")
    ?.addEventListener("click", () => envoyerCommande("arreter"));



//7 Export des mesures en CSV possible (lire les données en cas de probleme )

document
    .getElementById("exporter-donnees")
    ?.addEventListener("click", () => {
        if (!dernieresDonnees ||
            !Array.isArray(dernieresDonnees.points) ||
            dernieresDonnees.points.length === 0) {
            alert("Aucune mesure à exporter.");
            return;
        }

        const lignes = [
            "angle_deg,distance_m,x_m,y_m",
            ...dernieresDonnees.points.map((point) =>
                [
                    point.angle,
                    point.distance,
                    point.x,
                    point.y
                ].join(",")
            )
        ];

        const fichier = new Blob(
            [lignes.join("\n")],
            { type: "text/csv;charset=utf-8;" }
        );

        const url = URL.createObjectURL(fichier);
        const lien = document.createElement("a");

        lien.href = url;
        lien.download = "gyrobot_scan_lidar.csv";
        lien.click();

        URL.revokeObjectURL(url);
    });


//Actualisation du scan fixé à 250 ms pour le moment
actualiserScan();
setInterval(actualiserScan, 250);