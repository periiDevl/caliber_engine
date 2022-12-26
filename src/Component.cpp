#include "Component.h"

void::Object::AddObject(const char* location)
{
    OSWsave[objects] = Model(location, glm::vec3(0), glm::vec3(0), glm::quat(0, 0, 0, 0), glm::vec3(0));
    objects++;
    SuffleObjectsID();
}



void::Object::TRY_OBJ_RECOVERING_TEST()
{
    std::string line;
    std::ifstream OSWFile("projectname[OSW].caliber");
    int i = 0;

    while (std::getline(OSWFile, line)) // Use getline to check for the end of the file
    {
        try
        {
            OSWsave[i] = Model::From_string(line); // Convert the string to a model
            i++;
            objects++;
        }
        catch (const std::invalid_argument&)
        {
            // Handle the exception if the string cannot be converted to a float
            Error();
        }
    }

    std::cout << i + " OBJECTS RECOVERD" << std::endl;
}

void::Object::TRY_OBJ_SORTER_TEST()
{
    std::ofstream OSWFileWr("projectname[OSW].caliber");

    for (int i = 0; i < objects; i++)
    {
        std::string str = OSWsave[i].To_string();
        OSWFileWr << str << "\n";
    }
}

void::Object::SuffleObjectsID()
{
    std::vector<glm::vec3> allIDs;

    // Initialize allIDs with a list of all possible IDs
    for (int i = 0; i < objects; i++)
    {
        allIDs.push_back(glm::vec3(i, i, i));
    }

    // Shuffle the list of IDs
    std::random_device rd;
    std::mt19937 mt(rd());
    std::shuffle(allIDs.begin(), allIDs.end(), mt);

    for (int i = 0; i < objects; i++)
    {
        OSWsave[i].ID = allIDs[i];
    }
}


void::Object::TRY_SAFE_MODE()
{
    // Create a set to store the IDs
    std::set<glm::vec3> IDset;

    // Iterate through the objects in OSWsave
    for (int i = 0; i < OSWsave.size(); i++)
    {
        // If the ID is already in the set, return true
        if (IDset.count(OSWsave[i].ID) > 0)
        {
            printf("[SAFE TEST PASSED POORLY WAIT UNTIL ERRORS ARE FIXED]");
            SuffleObjectsID();
        }

        // Otherwise, add the ID to the set
        IDset.insert(OSWsave[i].ID);
    }

    // If no duplicate IDs were found, return false
    printf("[SAFE TEST PASSED SUCCESFULLY]");
}

Model Object::FindObjectID(GLFWwindow* window)
{
    // Get the mouse position in screen coordinates
    double mouseX, mouseY;
    glfwGetCursorPos(window, &mouseX, &mouseY);

    // Flip the y coordinate, since screen coordinates start at the top left,
    // but OpenGL coordinates start at the bottom left
    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);
    mouseY = viewport[3] - mouseY;

    // Read the color values of the pixel under the mouse cursor
    GLubyte pixel[4];
    glReadPixels(mouseX, mouseY, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, pixel);

    // The pixel array now contains the color values of the pixel under the mouse cursor
    // You can access the individual color components like this:
    float r = pixel[0] / 255.0f;
    float g = pixel[1] / 255.0f;
    float b = pixel[2] / 255.0f;

    // Convert the color values to an integer ID
    glm::vec3 ID = glm::vec3(r, g, b);

    // Search the objects in OSWsave for an object with the matching ID
    for (int i = 0; i < objects; i++)
    {
        if (OSWsave[i].ID == ID)
        {
            return OSWsave[i];
        }
    }


}


void::Object::Error()
{
    printf("[FAILED GENRATING/LOADING FILES]");
}