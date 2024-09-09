#define _GNU_SOURCE

#include <stdio.h>
#include <fcntl.h>   // File Control Definitions
#include <termios.h> // POSIX Terminal Control Definitions
#include <unistd.h>  // UNIX Standard Definitions
#include <errno.h>   // ERROR Number Definitions
#include <stdlib.h>
#include <string.h>
#include <signal.h>

const char *portTTY = "/dev/ttyS1"; // Port série à utiliser

// Fonction pour initialiser le port série
int init_serial(const char *port) {
    int fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        perror("Erreur! ouverture du port série");
        return -1;
    }
    printf("Ouverture de %s réussie.\n", port);

    struct termios SerialPortSettings;
    tcgetattr(fd, &SerialPortSettings);
    cfsetispeed(&SerialPortSettings, B115200);
    cfsetospeed(&SerialPortSettings, B115200);
    
    SerialPortSettings.c_cflag &= ~PARENB; // Pas de parité
    SerialPortSettings.c_cflag &= ~CSTOPB; // 1 bit de stop
    SerialPortSettings.c_cflag &= ~CSIZE;   // Masque pour définir la taille des données
    SerialPortSettings.c_cflag |= CS8;       // 8 bits de données
    SerialPortSettings.c_cflag &= ~CRTSCTS;  // Pas de contrôle de flux
    SerialPortSettings.c_cflag |= CREAD | CLOCAL; // Activer la réception

    SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY); // Pas de contrôle de flux

    // Mode canonique et désactivation de l'écho
    SerialPortSettings.c_lflag |= ICANON; 
    SerialPortSettings.c_lflag &= ~(ECHO | ECHOE | ISIG); 

    // VMIN = 1 caractère, VTIME = 0 (délai infini)
    SerialPortSettings.c_cc[VMIN] = 1;
    SerialPortSettings.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &SerialPortSettings) != 0) {
        perror("Erreur! configuration des attributs du port série");
        close(fd);
        return -1;
    }
    return fd;
}

int main() {
    int fd = init_serial(portTTY);
    if (fd < 0) return 1;

    pid_t pid = fork();

    if (pid < 0) {
        perror("Erreur lors de la création du processus");
        close(fd);
        return 1;
    }

    if (pid > 0) { // Processus Père
        printf("Je suis le processus Père, j'écrit sur la console (terminal) ce que j'entends sur le port série...\n");
        char read_buffer[256];
        while (1) {
            int bytes_read = read(fd, read_buffer, sizeof(read_buffer) - 1);
            if (bytes_read > 0) {
                read_buffer[bytes_read] = '\0'; // Terminer la chaîne
                printf("processus Père: nombres d'octets recus : %d --> %s\n", bytes_read, read_buffer);
                if (read_buffer[0] == '!') break; // Fin si on reçoit '!'
            }
        }
        printf("Fin du Père\n");
    } else { // Processus Fils
        printf("Je suis le processus Fils, j'écrit sur le port série ce que j'entends sur la console (terminal)...\n");
        char write_buffer[256];
        while (1) {
            fgets(write_buffer, sizeof(write_buffer), stdin);
            write(fd, write_buffer, strlen(write_buffer));
            if (write_buffer[0] == 'q') break; // Fin si on entre 'q'
        }
        printf("Fin du Fils\n");
    }

    close(fd);
    return 0;
}
