#include "narrowphase_manager.h"

namespace physics::internal {

//=======================================================
//     Emit rigid contact and wake up bodies if needed
//=======================================================
void NarrowphaseManager::emitRigidContact(
   ContactBatch& batch,
   OverlapHit& hit)
{
   // Primitive shapes only
   if (SAT::Result* geometry = 
      std::get_if<SAT::Result>(&hit.geometry)) 
   {
      ColliderEndpointRef& a = hit.pair.a;
      ColliderEndpointRef& b = hit.pair.b;

      tryExportExternalContact(hit.pair, *geometry);

      if (a.body->type == BodyType::Kinematic &&
         b.body->type == BodyType::Kinematic) {
         return;
      }

      ContactRuntime runtimeData = makeRuntimeData(
         a.body,
         b.body,
         a.collider,
         b.collider
      );

      Contact contact(
         a.bodyHandle,
         b.bodyHandle,
         runtimeData,
         geometry->normal
      );

      Contact* contactPtr = createManifold(contact, hit);
      batch.contacts.push_back(contactPtr);
   }
   // Mesh shapes involved
   else if (std::vector<SAT::Result>* geometry =
      std::get_if<std::vector<SAT::Result>>(&hit.geometry)) 
   {

   }
}

//=======================================================
//     Create contact manifold
//=======================================================
Contact* NarrowphaseManager::createManifold(
   Contact& contact,
   OverlapHit& hit)
{
   // Primitive shapes only
   if (SAT::Result* geometry = 
      std::get_if<SAT::Result>(&hit.geometry)) 
   {
      switch (hit.pair.shapePair) {
      case ShapePairKind::BoxBox:
         return collisionManifold->boxBox(
            contact, *contactCache, *geometry);

      case ShapePairKind::BoxSphere:
         return collisionManifold->boxSphere(
            contact, *contactCache, *geometry);

      case ShapePairKind::SphereSphere:
         return collisionManifold->sphereSphere(
            contact, *contactCache, *geometry);
      }
   }
   // Mesh shapes involved
   else if (std::vector<SAT::Result>* geometry = 
      std::get_if<std::vector<SAT::Result>>(&hit.geometry)) 
   {
      //switch (hit.pair.shapePair) {
      //case ShapePairKind::BoxMesh:
      //   return collisionManifold->boxMesh(
      //      contact, *contactCache, *geometry);

      //case ShapePairKind::SphereMesh:
      //   return collisionManifold->sphereMesh (
      //      contact, *contactCache, *geometry);

      //case ShapePairKind::MeshMesh:
      //   return collisionManifold->meshMesh(
      //      contact, *contactCache, *geometry);
      //}
   }

   return nullptr;
}

//=======================================================
//     Export external motion contacts for character controllers
//=======================================================
bool NarrowphaseManager::tryExportExternalContact(
   const ResolvedColliderPair& pair,
   const SAT::Result& sat)
{
   if (sat.hitType == SAT::HitType::Speculative) {
      return false;
   }

   if (!pair.a.body->reportContacts &&
      !pair.b.body->reportContacts) {
      return false;
   }

   externalContacts.emplace_back(
      pair.a.collider->bodyHandle,
      pair.b.collider->bodyHandle,
      sat.normal,
      sat.depth
   );

   return true;
}


}
