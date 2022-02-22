# ENE138 - Laboratório de Manipuladores Robóticos
 Atividades e trabalhos desenvolvidos para a disciplina Laboratório de Manipuladores Robóticos.

## Trabalho 1 - Cinemática Direta
Obtém o conjunto de posição (x,y) a partir de 5 conjuntos de ângulos (th1,th2) e aplica esses valores ao manipulador simulado (CopelliaSim).

## Trabalho 2 - Cinemática Inversa Algébrica
Calcula-se os valores de ângulos necessários (th1,th2) para se obter 3 conjuntos de posição (x,y). São obtidos os valores para elbow up e elbow down e aplicados ao simulador.

## Trabalho 3 - Cinemática Inversa Numérica
Calcula-se os valores de ângulos necessários (th1,th2) para se obter 5 conjuntos de posição (x,y) através do método gradiente descendente. São obtidos os valores para elbow up e elbow down e aplicados ao simulador.

## Trabalho 4
Calcula-se os valores de ângulos necessários (th1,th2) para se obter conjuntos de posição (x,y) através dos métodos Jacobiano Transposto e Jacobiano Inverso. Os valores são aplicados ao simulador.

## Trabalho 5
Utiliza as técnicas de campos potenciais para levar o robô de um ponto (x0,y0) até outro (xd,yd) no espaço de trabalho, desviando de obstáculos pontuais. Existem dois obstáculos, um no qual o robô consegue desviar e atingir o ponto objetivo e outro no qual o robô prende-se em um mínimo local.

## Trabalho 6 - Planejamento de Trajetórias
Utiliza as técnicas de planejamento de trajetória para levar o robô de um ponto (x0,y0) até outro (xd,yd) no espaço de trabalho, em um intervalo de tempo definido t. São utilizados 3 algoritmos
1. Polinômio de terceiro grau;
2. Polinômio de quinto grau;
3. Linear Segments with Parabolic Blends (LSPB)

O arquivo alterações_lspb.pdf demonstra algumas alterações feitas para a implementação desse método.
