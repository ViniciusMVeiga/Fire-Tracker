#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/geometry.hpp>


#include <iostream>
#include <vector>
#include <string>

using namespace cv;
using namespace std;


int main()
{
    for (int numero = 1; numero <= 8; numero++) {
        // 1. Define o nome e carrega a imagem
        string nomeArquivo = numero == 1
            ? "fogo.jpg"
            : "fogo" + to_string(numero) + ".jpg";

        Mat imagem = imread(nomeArquivo);

        if (imagem.empty()) {
            cerr << "Erro ao carregar: "
                 << nomeArquivo << endl;
            continue;
        }

        cout << "\nImagem: " << nomeArquivo << endl;

        // 2. Suavizacao
        Mat suavizada;
        GaussianBlur(imagem, suavizada, Size(5, 5), 0);

        // 3. Conversao para HSV
        Mat hsv;
        cvtColor(suavizada, hsv, COLOR_BGR2HSV);

        // 4. Filtragem das cores do fogo
        Mat mascara1, mascara2, mascara;

        inRange(hsv, Scalar(0, 100, 100),
                Scalar(25, 255, 255), mascara1);

        inRange(hsv, Scalar(170, 100, 100),
                Scalar(179, 255, 255), mascara2);

        bitwise_or(mascara1, mascara2, mascara);

        // 5. Operacoes morfologicas
        Mat elementoAbertura = getStructuringElement(
            MORPH_ELLIPSE, Size(3, 3));

        Mat elementoFechamento = getStructuringElement(
            MORPH_ELLIPSE, Size(5, 5));

        morphologyEx(mascara, mascara, MORPH_OPEN,
                     elementoAbertura);

        morphologyEx(mascara, mascara, MORPH_CLOSE,
                     elementoFechamento);

        // 6. Localizacao dos contornos
        vector<vector<Point>> contornos;

        findContours(mascara.clone(), contornos,
                     RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        // 7. Seleciona o maior contorno
        int maiorIndice = -1;
        double maiorArea = 0.0;

        for (int i = 0; i < static_cast<int>(contornos.size()); i++) {
            double area = contourArea(contornos[i]);

            if (area > maiorArea) {
                maiorArea = area;
                maiorIndice = i;
            }
        }

        // 8. Calcula e exibe o centro do fogo
        if (maiorIndice == -1 || maiorArea < 100.0) {
            cout << "Nenhuma regiao significativa de fogo detectada."
                 << endl;
        } else {
            Moments m = moments(contornos[maiorIndice]);

            if (m.m00 != 0.0) {
                int x = static_cast<int>(m.m10 / m.m00);
                int y = static_cast<int>(m.m01 / m.m00);

                cout << "Fogo detectado!" << endl;
                cout << "Area: " << maiorArea << " pixels" << endl;
                cout << "Centro: x = " << x
                     << ", y = " << y << endl;

                drawContours(imagem, contornos, maiorIndice,
                             Scalar(0, 255, 0), 2);

                circle(imagem, Point(x, y), 6,
                       Scalar(255, 0, 0), FILLED);

                putText(imagem, "Centro do fogo",
                        Point(x + 10, y - 10),
                        FONT_HERSHEY_SIMPLEX, 0.6,
                        Scalar(255, 0, 0), 2);
            }
        }

        // 9. Exibe os resultados
        imshow("Deteccao de fogo", imagem);
        imshow("Mascara binaria", mascara);

        cout << "Pressione uma tecla na janela da imagem "
             << "para continuar..." << endl;

        int tecla = waitKey(0);

        destroyAllWindows();

        if (tecla == 27) { // ESC encerra os testes
            break;
        }
    }

    cout << "\nTestes finalizados." << endl;

    return 0;
}