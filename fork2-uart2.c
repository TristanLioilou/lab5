#define _GNU_SOURCE
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/wait.h>

const char *portTTY = "/dev/ttyS1";

int init_serial(const char *port) {
    int fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        perror("Erreur! ouverture du port");
        return -1;
    }

    struct termios SerialPortSettings;
    tcgetattr(fd, &SerialPortSettings);
    cfsetispeed(&SerialPortSettings, B115200);
    cfsetospeed(&SerialPortSettings, B115200);

    SerialPortSettings.c_cflag &= ~PARENB;
    SerialPortSettings.c_cflag &= ~CSTOPB;
    SerialPortSettings.c_cflag &= ~CSIZE;
    SerialPortSettings.c_cflag |= CS8;
    SerialPortSettings.c_cflag &= ~CRTSCTS;
    SerialPortSettings.c_cflag |= CREAD | CLOCAL;

    SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);
    SerialPortSettings.c_lflag |= (ICANON | ECHO);
    SerialPortSettings.c_oflag &= ~OPOST;

    // Set VMIN to 0 for non-blocking reads
    SerialPortSettings.c_cc[VMIN] = 0;  
    SerialPortSettings.c_cc[VTIME] = 0; // set VTIME to 0 for no timeout

    if (tcsetattr(fd, TCSANOW, &SerialPortSettings) != 0) {
        perror("Erreur! configuration des attributs du port serie");
        return -1;
    }

    return fd;
}

void read_from_serial(int fd, int pipe_fd) {
    char read_buffer[256];
    while (1) {
        int bytes_read = read(fd, read_buffer, sizeof(read_buffer));
        if (bytes_read > 0) {
            printf("Processus de Lecture: nombre d'octets reçus : %d --> %s\n", bytes_read, read_buffer);
            write(pipe_fd, read_buffer, bytes_read); // Envoyer les données lues au pipe
            if (read_buffer[0] == '!') {
                break; // Exit on receiving '!'
            }
        }
        usleep(100000); // Sleep for 100 ms
    }
    printf("Fin du processus de Lecture\n");
}

void write_to_serial(int fd, int pipe_fd) {
    char write_buffer[256];
    while (1) {
        read(pipe_fd, write_buffer, sizeof(write_buffer)); // Lire depuis le pipe
        write(fd, write_buffer, sizeof(write_buffer));
        if (write_buffer[0] == 'q') {
            break; // Exit on sending 'q'
        }
    }
    printf("Fin du processus d'Ecriture\n");
}

int main(void) {
    int fd_portUART = init_serial(portTTY);
    if (fd_portUART == -1) {
        return 1;
    }

    int pipe_read_to_parent[2];
    int pipe_parent_to_write[2];

    // Créer les pipes
    pipe(pipe_read_to_parent);
    pipe(pipe_parent_to_write);

    pid_t pid_read = fork();
    if (pid_read < 0) {
        perror("Erreur lors du fork pour la lecture");
        return 1;
    } else if (pid_read == 0) {
        // Processus enfant pour la lecture
        close(pipe_read_to_parent[0]); // Ferme le côté lecture du pipe
        read_from_serial(fd_portUART, pipe_read_to_parent[1]);
        close(pipe_read_to_parent[1]); // Ferme le côté écriture du pipe
        _exit(0);
    }

    pid_t pid_write = fork();
    if (pid_write < 0) {
        perror("Erreur lors du fork pour l'écriture");
        return 1;
    } else if (pid_write == 0) {
        // Processus enfant pour l'écriture
        close(pipe_parent_to_write[1]); // Ferme le côté écriture du pipe
        write_to_serial(fd_portUART, pipe_parent_to_write[0]);
        close(pipe_parent_to_write[0]); // Ferme le côté lecture du pipe
        _exit(0);
    }

    // Processus principal
    int n = 1;
    char command[256];
    while (n < 10) {
        printf("1 faire quelques trucs...\n");
        printf("Entrez une commande à envoyer à l'écriture (ou 'q' pour quitter) : ");
        fgets(command, sizeof(command), stdin);
        write(pipe_parent_to_write[1], command, sizeof(command)); // Écrire dans le pipe pour l'écriture
        n++;
        sleep(3);
    }

    // Signaler la fin
    write(pipe_parent_to_write[1], "q", 1); // Envoyer 'q' pour terminer l'écriture
    wait(NULL); // Attendre la fin du processus de lecture
    wait(NULL); // Attendre la fin du processus d'écriture
    close(fd_portUART); // Fermer le port série
    printf("Fin du processus Principal\n");

    return 0;
}
